"""
RAG Query Logic using OpenAI Agents SDK
"""

from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import OpenAI
from config import get_settings

settings = get_settings()

class RAGQuery:
    def __init__(self):
        # Fallback to local storage if no URL provided
        if settings.qdrant_url and "http" in settings.qdrant_url:
            self.qdrant = QdrantClient(
                url=settings.qdrant_url, 
                api_key=settings.qdrant_api_key
            )
        else:
            self.qdrant = QdrantClient(path=settings.qdrant_path)

        self.openai = OpenAI(api_key=settings.openai_api_key)
        self.collection_name = settings.qdrant_collection
        
    def query(self, question: str, context_text: Optional[str] = None) -> Dict:
        """
        Answer a question using RAG.
        If context_text is provided (from text selection), it is boosted in relevance.
        """
        
        # 1. Generate embedding for the question
        response = self.openai.embeddings.create(
            input=question,
            model=settings.openai_embedding_model
        )
        question_vector = response.data[0].embedding
        
        # 2. Search Qdrant
        search_results = self.qdrant.search(
            collection_name=self.collection_name,
            query_vector=question_vector,
            limit=5
        )
        
        # 3. Construct Context
        retrieved_context = ""
        citations = []
        
        for result in search_results:
            text = result.payload.get("content", "")
            file = result.payload.get("file", "unknown")
            header = result.payload.get("header", "unknown")
            
            retrieved_context += f"\n---\nSource: {file} > {header}\n{text}\n"
            citations.append(f"{file} ({header})")
            
        # Add user selected text if available
        if context_text:
            retrieved_context = f"\n---\nUSER SELECTED CONTEXT:\n{context_text}\n" + retrieved_context
            
        # 4. Generate Answer
        system_prompt = """You are a helpful AI teaching assistant for a Physical AI & Robotics textbook.
        Use the provided context to answer the student's question.
        If the answer is not in the context, say you don't know but offer general knowledge if applicable, noting it's not from the book.
        Keep answers concise and educational.
        """
        
        completion = self.openai.chat.completions.create(
            model=settings.openai_model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Context:\n{retrieved_context}\n\nQuestion: {question}"}
            ]
        )
        
        answer = completion.choices[0].message.content
        
        return {
            "answer": answer,
            "citations": list(set(citations)) # Unique citations
        }
