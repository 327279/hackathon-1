"""
User Profile and Onboarding Routes
"""

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from pydantic import BaseModel
from typing import Optional, List
from database import get_db, User, UserProfile
from .auth import get_current_user

router = APIRouter(prefix="/api/profile", tags=["profile"])

class ProfileUpdate(BaseModel):
    role: Optional[str] = None
    interests: Optional[List[str]] = None
    experience_level: Optional[str] = None # Mapping to software_experience for onboarding
    software_experience: Optional[str] = None
    hardware_experience: Optional[str] = None
    programming_languages: Optional[str] = None
    goal: Optional[str] = None

class ProfileResponse(BaseModel):
    role: Optional[str]
    interests: Optional[str] # Stored as comma-separated
    software_experience: Optional[str]
    hardware_experience: Optional[str]
    programming_languages: Optional[str]
    goal: Optional[str]

@router.get("/", response_model=ProfileResponse)
async def get_profile(
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == current_user.id))
    profile = result.scalar_one_or_none()
    
    if not profile:
        # Should not happen as we create one on signup, but good safety
        profile = UserProfile(user_id=current_user.id)
        db.add(profile)
        await db.commit()
        await db.refresh(profile)
        
    return profile

@router.put("/", response_model=ProfileResponse)
async def update_profile(
    profile_data: ProfileUpdate,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == current_user.id))
    profile = result.scalar_one_or_none()
    
    if not profile:
        profile = UserProfile(user_id=current_user.id)
        db.add(profile)
    
    if profile_data.role is not None:
        profile.role = profile_data.role
    if profile_data.interests is not None:
        profile.interests = ",".join(profile_data.interests)
    if profile_data.experience_level is not None:
        profile.software_experience = profile_data.experience_level
        
    if profile_data.software_experience is not None:
        profile.software_experience = profile_data.software_experience
    if profile_data.hardware_experience is not None:
        profile.hardware_experience = profile_data.hardware_experience
    if profile_data.programming_languages is not None:
        profile.programming_languages = profile_data.programming_languages
    if profile_data.goal is not None:
        profile.goal = profile_data.goal
        
    await db.commit()
    await db.refresh(profile)
    return profile
