"""
RAG Ingestion Pipeline using OpenAI Agents SDK
"""

import os
import glob
from typing import List, Dict
from qdrant_client import QdrantClient
from qdrant_client.http import models
import google.generativeai as genai
from config import get_settings

settings = get_settings()

class IngestionPipeline:
    def __init__(self):
        # Fallback to local storage if no URL provided or connection fails
        if settings.qdrant_url and "http" in settings.qdrant_url:
            print(f"Connecting to Qdrant Cloud: {settings.qdrant_url}")
            self.qdrant = QdrantClient(
                url=settings.qdrant_url, 
                api_key=settings.qdrant_api_key
            )
        else:
            print(f"Using Local Qdrant Storage: {settings.qdrant_path}")
            self.qdrant = QdrantClient(path=settings.qdrant_path)

        genai.configure(api_key=settings.gemini_api_key)
        self.collection_name = settings.qdrant_collection
        
    def init_collection(self):
        """Create Qdrant collection if not exists"""
        collections = self.qdrant.get_collections()
        exists = any(c.name == self.collection_name for c in collections.collections)
        
        if not exists:
            self.qdrant.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=768,  # Gemini text-embedding-004 dimension
                    distance=models.Distance.COSINE
                )
            )
            print(f"Created collection: {self.collection_name}")
            
    def process_file(self, file_path: str) -> List[Dict]:
        """Read and chunk a markdown file"""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
        # Simple chunking by headers for now
        # In production, use a proper recursive character splitter
        chunks = []
        lines = content.split('\n')
        current_chunk = []
        current_header = "Introduction"
        
        for line in lines:
            if line.startswith('#'):
                if current_chunk:
                    chunks.append({
                        "content": '\n'.join(current_chunk),
                        "header": current_header,
                        "file": os.path.basename(file_path)
                    })
                    current_chunk = []
                current_header = line.strip('#').strip()
            
            current_chunk.append(line)
            
        if current_chunk:
            chunks.append({
                "content": '\n'.join(current_chunk),
                "header": current_header,
                "file": os.path.basename(file_path)
            })
            
        return chunks

    def ingest_directory(self, docs_path: str):
        """Ingest all .md files from docs directory"""
        self.init_collection()
        
        files = glob.glob(os.path.join(docs_path, "**/*.md"), recursive=True)
        print(f"Found {len(files)} files to ingest...")
        
        for file_path in files:
            chunks = self.process_file(file_path)
            
            for i, chunk in enumerate(chunks):
                try:
                    # Generate embedding with retry logic
                    import time
                    from tenacity import retry, stop_after_attempt, wait_exponential
                    
                    @retry(stop=stop_after_attempt(5), wait=wait_exponential(multiplier=1, min=4, max=30))
                    def generate_embedding(content):
                        return genai.embed_content(
                            model=settings.gemini_embedding_model,
                            content=content,
                            task_type="retrieval_document"
                        )
                    
                    embedding_result = generate_embedding(chunk['content'])
                    embedding = embedding_result['embedding']
                    
                    # Upsert to Qdrant
                    self.qdrant.upsert(
                        collection_name=self.collection_name,
                        points=[
                            models.PointStruct(
                                id=str(os.urandom(16).hex()), # Use random ID to avoid collision
                                vector=embedding,
                                payload={
                                    "content": chunk['content'],
                                    "header": chunk['header'],
                                    "file": chunk['file'],
                                    "path": file_path
                                }
                            )
                        ]
                    )
                    
                    # Increased delay to avoid rate limits
                    time.sleep(1.5)
                    
                except Exception as e:
                    print(f"Error ingesting chunk from {file_path}: {e}")
                    continue
                    
            print(f"Ingested {file_path}")

if __name__ == "__main__":
    # Create a dummy .env for testing if needed
    pipeline = IngestionPipeline()
    # Assuming run from backend directory
    pipeline.ingest_directory("../docs")
