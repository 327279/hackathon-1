import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

interface Message {
    id: string;
    role: 'user' | 'assistant';
    content: string;
}

export default function ChatBot(): JSX.Element {
    const { siteConfig } = useDocusaurusContext();
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState<Message[]>([
        { id: '1', role: 'assistant', content: 'Hi! I can help you with Physical AI. Ask me anything about the textbook!' }
    ]);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const [selectedContext, setSelectedContext] = useState<string | null>(null);
    const messagesEndRef = useRef<HTMLDivElement>(null);

    // Scroll to bottom on new message
    useEffect(() => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, [messages]);

    // Handle text selection
    useEffect(() => {
        const handleSelection = () => {
            const selection = window.getSelection();
            if (selection && selection.toString().trim().length > 0) {
                // Only set context if selection is within the documentation content
                // This is a naive check; ideally we'd check if the selection anchorNode is inside the markdown container
                const text = selection.toString().trim();
                if (text.length < 500) { // Limit context length
                    // We don't automatically set it, but we could show a tooltip.
                    // For now, let's just listen for a specific event or store it
                }
            }
        };

        document.addEventListener('mouseup', handleSelection);
        return () => document.removeEventListener('mouseup', handleSelection);
    }, []);

    // Listen for custom event from text selection popover (implemented later)
    useEffect(() => {
        const handleContextEvent = (e: CustomEvent) => {
            setIsOpen(true);
            setSelectedContext(e.detail.text);
        };

        window.addEventListener('chatbot-context', handleContextEvent as EventListener);
        return () => window.removeEventListener('chatbot-context', handleContextEvent as EventListener);
    }, []);

    const sendMessage = async () => {
        if (!input.trim() || isLoading) return;

        const userMessage: Message = {
            id: Date.now().toString(),
            role: 'user',
            content: input
        };

        setMessages(prev => [...prev, userMessage]);
        setInput('');
        setIsLoading(true);

        try {
            // API base URL - production backend
            const apiBase = 'https://hackathon-1-lake-five.vercel.app';

            const response = await fetch(`${apiBase}/api/chat/`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    message: userMessage.content,
                    selected_text: selectedContext
                }),
            });

            if (!response.ok) throw new Error('Network response was not ok');

            const data = await response.json();

            const botMessage: Message = {
                id: (Date.now() + 1).toString(),
                role: 'assistant',
                content: data.response
            };

            setMessages(prev => [...prev, botMessage]);
            setSelectedContext(null); // Clear context after use
        } catch (error) {
            console.error('Error:', error);
            const errorMessage: Message = {
                id: (Date.now() + 1).toString(),
                role: 'assistant',
                content: 'Sorry, I encountered an error connecting to the brain. Please ensure the backend is running.'
            };
            setMessages(prev => [...prev, errorMessage]);
        } finally {
            setIsLoading(false);
        }
    };

    const handleKeyPress = (e: React.KeyboardEvent) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendMessage();
        }
    };

    return (
        <>
            <button
                className={`${styles.toggleButton} ${isOpen ? styles.hidden : ''}`}
                onClick={() => setIsOpen(true)}
                style={{ display: isOpen ? 'none' : 'flex' }}
            >
                🤖
            </button>

            <div className={`${styles.chatContainer} ${!isOpen ? styles.closed : ''}`}>
                <div className={styles.chatHeader}>
                    <span>AI Assistant</span>
                    <button className={styles.closeButton} onClick={() => setIsOpen(false)}>×</button>
                </div>

                <div className={styles.messagesContainer}>
                    {messages.map((msg) => (
                        <div key={msg.id} className={`${styles.message} ${msg.role === 'user' ? styles.userMessage : styles.botMessage}`}>
                            {msg.content}
                        </div>
                    ))}
                    {isLoading && <div className={`${styles.message} ${styles.botMessage}`}>Thinking...</div>}
                    <div ref={messagesEndRef} />
                </div>

                {selectedContext && (
                    <div className={styles.contextIndicator}>
                        <span>Reference: "{selectedContext.substring(0, 30)}..."</span>
                        <button className={styles.clearContext} onClick={() => setSelectedContext(null)}>×</button>
                    </div>
                )}

                <div className={styles.inputContainer}>
                    <input
                        className={styles.input}
                        value={input}
                        onChange={(e) => setInput(e.target.value)}
                        onKeyPress={handleKeyPress}
                        placeholder="Ask a question..."
                        disabled={isLoading}
                    />
                    <button
                        className={styles.sendButton}
                        onClick={sendMessage}
                        disabled={!input.trim() || isLoading}
                    >
                        ➤
                    </button>
                </div>
            </div>
        </>
    );
}
