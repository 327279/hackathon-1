import React, { useState } from 'react';

export const TranslateButton = ({ content, onTranslated }) => {
    const [loading, setLoading] = useState(false);
    const [active, setActive] = useState(false);

    const handleTranslate = async () => {
        if (active) {
            // Toggle back to original (handled by parent usually, or we just reload)
            window.location.reload();
            return;
        }

        setLoading(true);
        try {
            const response = await fetch('http://localhost:8000/api/translate/', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ content, target_language: 'Urdu' })
            });

            const data = await response.json();
            if (data.translated_content) {
                onTranslated(data.translated_content);
                setActive(true);
            }
        } catch (err) {
            console.error(err);
            alert('Translation failed');
        } finally {
            setLoading(false);
        }
    };

    return (
        <button
            onClick={handleTranslate}
            className={`button button--${active ? 'secondary' : 'primary'}`}
            style={{ margin: '0 0.5rem' }}
            disabled={loading}
        >
            {loading ? 'Translating...' : (active ? 'Show Original' : 'Translate to Urdu 🇵🇰')}
        </button>
    );
};
