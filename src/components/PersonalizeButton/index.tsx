import React, { useState } from 'react';

export const PersonalizeButton = ({ content, title, onPersonalized }) => {
    const [loading, setLoading] = useState(false);
    const [active, setActive] = useState(false);

    const handlePersonalize = async () => {
        setLoading(true);
        const token = localStorage.getItem('token');

        if (!token) {
            alert("Please log in to use AI Personalization");
            setLoading(false);
            return;
        }

        try {
            const response = await fetch('http://localhost:8000/api/personalize/', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'Authorization': `Bearer ${token}`
                },
                body: JSON.stringify({ content, chapter_title: title })
            });

            const data = await response.json();
            if (data.personalized_content) {
                onPersonalized(data.personalized_content);
                setActive(true);
            }
        } catch (err) {
            console.error(err);
            alert('Personalization failed');
        } finally {
            setLoading(false);
        }
    };

    return (
        <button
            onClick={handlePersonalize}
            className={`button button--${active ? 'success' : 'info'}`}
            style={{ margin: '0 0.5rem' }}
            disabled={loading || active}
        >
            {loading ? 'Adapting...' : (active ? 'Personalized for You ✨' : 'Personalize Content 🧠')}
        </button>
    );
};
