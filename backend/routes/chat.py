"""
Chat API Routes with graceful fallback
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, List
import os

router = APIRouter(prefix="/api/chat", tags=["chat"])

# Try to initialize RAG engine, but don't fail if dependencies are missing
rag_engine = None
try:
    from rag.query import RAGQuery
    rag_engine = RAGQuery()
    print("✅ RAG engine initialized successfully")
except Exception as e:
    print(f"⚠️ RAG engine not available: {e}")
    print("Chat will use fallback mode (direct OpenAI without RAG)")

class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None
    history: Optional[List[dict]] = None

class ChatResponse(BaseModel):
    response: str
    citations: List[str]

@router.post("/", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    try:
        # Try RAG first
        if rag_engine:
            result = rag_engine.query(
                question=request.message,
                context_text=request.selected_text
            )
            return ChatResponse(
                response=result["answer"],
                citations=result["citations"]
            )
        
        # Fallback: Direct Gemini without RAG
        gemini_key = os.getenv("GEMINI_API_KEY")
        if gemini_key:
            import google.generativeai as genai
            genai.configure(api_key=gemini_key)
            model = genai.GenerativeModel('gemini-2.0-flash')
            
            system_prompt = """You are a helpful AI teaching assistant for a Physical AI & Robotics textbook.
            Answer questions about ROS 2, robotics simulation, NVIDIA Isaac, and Vision-Language-Action models.
            Keep answers concise and educational."""
            
            context = ""
            if request.selected_text:
                context = f"\n\nUser selected this text for context: {request.selected_text}"
            
            prompt = f"{system_prompt}\n\nQuestion: {request.message}{context}"
            response = model.generate_content(prompt)
            
            return ChatResponse(
                response=response.text,
                citations=["Direct AI response (RAG not configured)"]
            )
        
        # No API key available
        return ChatResponse(
            response="I'm sorry, but the AI backend is not fully configured. Please ensure the GEMINI_API_KEY environment variable is set in your Vercel deployment.",
            citations=[]
        )
        
    except Exception as e:
        print(f"Error in chat endpoint: {e}")
        # Return a helpful error message instead of 500
        return ChatResponse(
            response=f"I encountered an error: {str(e)}. Please check the backend configuration.",
            citations=[]
        )
