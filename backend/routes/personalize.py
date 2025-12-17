"""
Personalization Routes
"""

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from database import get_db, User, UserProfile
from .auth import get_current_user
from config import get_settings
import google.generativeai as genai

router = APIRouter(prefix="/api/personalize", tags=["personalize"])
settings = get_settings()
genai.configure(api_key=settings.gemini_api_key)

class PersonalizeRequest(BaseModel):
    content: str
    chapter_title: str

class PersonalizeResponse(BaseModel):
    personalized_content: str

@router.post("/", response_model=PersonalizeResponse)
async def personalize_content(
    request: PersonalizeRequest,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    # Fetch user profile
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == current_user.id))
    profile = result.scalar_one_or_none()
    
    if not profile:
        raise HTTPException(status_code=400, detail="User profile not completion")
        
    software_exp = profile.software_experience or "intermediate"
    hardware_exp = profile.hardware_experience or "none"
    
    # Construct prompt
    system_prompt = f"""You are an expert tutor adapting a textbook for a student.
    Student Profile:
    - Software Experience: {software_exp}
    - Hardware Experience: {hardware_exp}
    
    Task: Rewrite the provided content to be more suitable for this student.
    - If beginner/none: Use simpler analogies, explain jargon.
    - If advanced: Be concise, focus on implementation details.
    - Keep the core technical accuracy.
    - Maintain Markdown formatting.
    """
    
    model = genai.GenerativeModel(settings.gemini_model)
    
    completion = model.generate_content(
        f"{system_prompt}\n\nTitle: {request.chapter_title}\n\nContent:\n{request.content}"
    )
    
    return PersonalizeResponse(personalized_content=completion.text)
