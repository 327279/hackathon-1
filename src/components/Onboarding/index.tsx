import React, { useState } from 'react';
import styles from './styles.module.css'; // Reusing similar styles or create new ones

export const OnboardingModal = ({ isOpen, onClose }) => {
    const [role, setRole] = useState('student');
    const [experience, setExperience] = useState('beginner');
    const [interests, setInterests] = useState([]);
    const [loading, setLoading] = useState(false);

    if (!isOpen) return null;

    const handleInterestToggle = (interest) => {
        if (interests.includes(interest)) {
            setInterests(interests.filter(i => i !== interest));
        } else {
            setInterests([...interests, interest]);
        }
    };

    const handleSubmit = async (e) => {
        e.preventDefault();
        setLoading(true);
        const token = localStorage.getItem('token');

        try {
            const response = await fetch('http://localhost:8000/api/profile/', {
                method: 'PUT',
                headers: {
                    'Content-Type': 'application/json',
                    'Authorization': `Bearer ${token}`
                },
                body: JSON.stringify({
                    role,
                    experience_level: experience,
                    interests
                }),
            });

            if (!response.ok) throw new Error('Failed to update profile');

            onClose();
            // Maybe trigger a confetti or welcome message
        } catch (err) {
            console.error(err);
            alert('Error updating profile');
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className="authOverlay" style={{
            position: 'fixed', top: 0, left: 0, width: '100vw', height: '100vh',
            background: 'rgba(0,0,0,0.8)', zIndex: 1000, display: 'flex', justifyContent: 'center', alignItems: 'center'
        }}>
            <div className="glass-panel" style={{ padding: '2rem', maxWidth: '500px', width: '100%' }}>
                <h2 style={{ textAlign: 'center', marginBottom: '2rem' }}>Personalize Your Journey</h2>
                <form onSubmit={handleSubmit}>

                    <div style={{ marginBottom: '1.5rem' }}>
                        <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 600 }}>Role</label>
                        <select
                            value={role}
                            onChange={(e) => setRole(e.target.value)}
                            style={{ width: '100%', padding: '0.8rem', borderRadius: '8px', background: 'rgba(255,255,255,0.05)', color: 'var(--ifm-color-content)', border: '1px solid rgba(255,255,255,0.1)' }}
                        >
                            <option value="student">Student</option>
                            <option value="researcher">Researcher</option>
                            <option value="hobbyist">Hobbyist</option>
                            <option value="professional">Professional</option>
                        </select>
                    </div>

                    <div style={{ marginBottom: '1.5rem' }}>
                        <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 600 }}>Experience Level</label>
                        <div style={{ display: 'flex', gap: '1rem' }}>
                            {['beginner', 'intermediate', 'advanced'].map((level) => (
                                <div
                                    key={level}
                                    onClick={() => setExperience(level)}
                                    style={{
                                        padding: '0.5rem 1rem',
                                        borderRadius: '20px',
                                        cursor: 'pointer',
                                        background: experience === level ? 'var(--ifm-color-primary)' : 'rgba(255,255,255,0.05)',
                                        border: `1px solid ${experience === level ? 'var(--ifm-color-primary)' : 'rgba(255,255,255,0.1)'}`,
                                        textTransform: 'capitalize'
                                    }}
                                >
                                    {level}
                                </div>
                            ))}
                        </div>
                    </div>

                    <div style={{ marginBottom: '2rem' }}>
                        <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 600 }}>Interests</label>
                        <div style={{ display: 'flex', flexWrap: 'wrap', gap: '0.5rem' }}>
                            {['ROS 2', 'Computer Vision', 'Reinforcement Learning', 'Navigation', 'Manipulation'].map((interest) => (
                                <div
                                    key={interest}
                                    onClick={() => handleInterestToggle(interest)}
                                    style={{
                                        padding: '0.5rem 1rem',
                                        borderRadius: '20px',
                                        cursor: 'pointer',
                                        fontSize: '0.9rem',
                                        background: interests.includes(interest) ? 'var(--ifm-color-primary)' : 'rgba(255,255,255,0.05)',
                                        border: `1px solid ${interests.includes(interest) ? 'var(--ifm-color-primary)' : 'rgba(255,255,255,0.1)'}`
                                    }}
                                >
                                    {interest}
                                </div>
                            ))}
                        </div>
                    </div>

                    <button type="submit" className="button button--primary button--lg" style={{ width: '100%' }} disabled={loading}>
                        {loading ? 'Saving...' : 'Start Learning'}
                    </button>
                </form>
            </div>
        </div>
    );
};
