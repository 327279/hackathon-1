"""
Database models and connection setup for Physical AI Textbook
"""

from sqlalchemy import Column, Integer, String, Boolean, ForeignKey, DateTime, Text, ARRAY
from sqlalchemy.orm import relationship, declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from datetime import datetime
import uuid
from config import get_settings

settings = get_settings()

# Use SQLite for local development if no DB URL provided, else Postgres
DATABASE_URL = settings.database_url if settings.database_url else "sqlite+aiosqlite:///./test.db"

engine = create_async_engine(DATABASE_URL, echo=settings.debug)
AsyncSessionLocal = sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)
Base = declarative_base()

class User(Base):
    __tablename__ = "users"
    
    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    email = Column(String, unique=True, index=True)
    hashed_password = Column(String)
    is_active = Column(Boolean, default=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    
    profile = relationship("UserProfile", back_populates="user", uselist=False)
    chat_history = relationship("ChatMessage", back_populates="user")

class UserProfile(Base):
    __tablename__ = "user_profiles"
    
    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, ForeignKey("users.id"))
    role = Column(String) # student, researcher, etc.
    interests = Column(String) # Comma-separated list
    software_experience = Column(Text) # Beginner, Intermediate, Advanced
    hardware_experience = Column(Text) # None, Arduino, Raspberry Pi, ROS Robots
    programming_languages = Column(String) # Comma-separated list
    goal = Column(Text)
    
    user = relationship("User", back_populates="profile")

class ChatMessage(Base):
    __tablename__ = "chat_messages"
    
    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, ForeignKey("users.id"))
    role = Column(String) # user, assistant
    content = Column(Text)
    created_at = Column(DateTime, default=datetime.utcnow)
    
    user = relationship("User", back_populates="chat_history")

# Dependency
async def get_db():
    async with AsyncSessionLocal() as session:
        yield session

async def init_db():
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
