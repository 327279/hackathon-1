import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

// This component handles the "Ask AI" button that appears when text is selected
export default function TextSelectionHandler() {
    const [position, setPosition] = useState<{ top: number, left: number } | null>(null);
    const [selectedText, setSelectedText] = useState('');

    useEffect(() => {
        const handleSelection = () => {
            const selection = window.getSelection();
            if (!selection || selection.toString().trim().length === 0) {
                setPosition(null);
                return;
            }

            const text = selection.toString().trim();
            // Only show if selection is substantial but not huge
            if (text.length > 5 && text.length < 1000) {
                const range = selection.getRangeAt(0);
                const rect = range.getBoundingClientRect();

                // Position above the selection
                setPosition({
                    top: rect.top + window.scrollY - 40,
                    left: rect.left + window.scrollX + (rect.width / 2) - 50
                });
                setSelectedText(text);
            } else {
                setPosition(null);
            }
        };

        const handleMouseDown = (e: MouseEvent) => {
            // Hide if clicking outside the popover (and not on the popover itself)
            // Implementation detail: we rely on blur or simple click away
            // For now, let's keep it simple: any click clears unless it's the button
        };

        document.addEventListener('mouseup', handleSelection);
        // document.addEventListener('mousedown', handleMouseDown);

        return () => {
            document.removeEventListener('mouseup', handleSelection);
            // document.removeEventListener('mousedown', handleMouseDown);
        };
    }, []);

    const handleAskAI = (e: React.MouseEvent) => {
        e.stopPropagation();
        e.preventDefault();

        // Dispatch custom event that ChatBot listens to
        const event = new CustomEvent('chatbot-context', {
            detail: { text: selectedText }
        });
        window.dispatchEvent(event);

        // Clear selection/popover
        window.getSelection()?.removeAllRanges();
        setPosition(null);
    };

    if (!position) return null;

    return (
        <button
            onClick={handleAskAI}
            style={{
                position: 'absolute',
                top: position.top,
                left: position.left,
                zIndex: 200,
                backgroundColor: 'var(--ifm-color-primary)',
                color: 'white',
                border: 'none',
                borderRadius: '20px',
                padding: '5px 15px',
                cursor: 'pointer',
                boxShadow: '0 2px 8px rgba(0,0,0,0.2)',
                fontWeight: 'bold',
                fontSize: '14px',
                transform: 'translateY(0)',
                transition: 'transform 0.1s',
                animation: 'popIn 0.2s cubic-bezier(0.175, 0.885, 0.32, 1.275)'
            }}
        >
            Ask AI ✨
        </button>
    );
}

// Add global style for animation
if (typeof document !== 'undefined') {
    const style = document.createElement('style');
    style.innerHTML = `
        @keyframes popIn {
            from { transform: scale(0); }
            to { transform: scale(1); }
        }
    `;
    document.head.appendChild(style);
}
