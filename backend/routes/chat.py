"""
Chat API Routes
"""

from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional, List
from rag.query import RAGQuery

router = APIRouter(prefix="/api/chat", tags=["chat"])

# Initialize RAG engine (singleton for now)
rag_engine = RAGQuery()

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
        result = rag_engine.query(
            question=request.message,
            context_text=request.selected_text
        )
        
        return ChatResponse(
            response=result["answer"],
            citations=result["citations"]
        )
    except Exception as e:
        print(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))
