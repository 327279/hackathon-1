"""
Physical AI & Humanoid Robotics Textbook - FastAPI Backend
Following Spec-Kit Plus methodology
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import os
from dotenv import load_dotenv

load_dotenv()

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup and shutdown events"""
    print("🚀 Starting Physical AI Textbook API...")
    yield
    print("👋 Shutting down...")

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="RAG Chatbot, Personalization, and Translation services",
    version="1.0.0",
    lifespan=lifespan
)

# CORS configuration - allow all origins for hackathon demo
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for hackathon
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "service": "physical-ai-textbook-api"}

@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "Physical AI & Humanoid Robotics Textbook API",
        "docs": "/docs",
        "health": "/health"
    }

# Route imports
from routes import chat, auth, profile, personalize, translate

app.include_router(chat.router)
app.include_router(auth.router)
app.include_router(profile.router)
app.include_router(personalize.router)
app.include_router(translate.router)
