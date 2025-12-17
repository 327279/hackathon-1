import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

export const AuthModal = ({ isOpen, onClose }) => {
    const [isLogin, setIsLogin] = useState(true);
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');

    if (!isOpen) return null;

    const handleSubmit = async (e) => {
        e.preventDefault();
        setLoading(true);
        setError('');

        try {
            if (isLogin) {
                // Login Logic
                const formData = new FormData();
                formData.append('username', email);
                formData.append('password', password);

                const response = await fetch('https://hackathon-1-lake-five.vercel.app/api/auth/token', {
                    method: 'POST',
                    body: formData,
                });

                if (!response.ok) {
                    throw new Error('Login failed. Check credentials.');
                }

                const data = await response.json();
                localStorage.setItem('token', data.access_token);
                // Dispatch event for other components
                window.dispatchEvent(new Event('auth-change'));
                onClose();
            } else {
                // Signup Logic
                const response = await fetch('https://hackathon-1-lake-five.vercel.app/api/auth/signup', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ email, password }),
                });

                if (!response.ok) {
                    const errData = await response.json();
                    throw new Error(errData.detail || 'Signup failed');
                }

                // Auto login after signup
                setIsLogin(true);
                setError('Signup successful! Please log in.');
                setLoading(false);
                return; // specific return to avoid closing immediately so they see the valid msg
            }
        } catch (err) {
            setError(err.message);
        } finally {
            if (isLogin) setLoading(false);
        }
    };

    return (
        <div className={styles.authOverlay}>
            <div className={`${styles.authModal} glass-panel`}>
                <button className={styles.closeButton} onClick={onClose}>×</button>
                <h2 className={styles.title}>{isLogin ? 'Welcome Back' : 'Join the Future'}</h2>

                {error && <div style={{ color: '#ef4444', marginBottom: '1rem', textAlign: 'center' }}>{error}</div>}

                <form onSubmit={handleSubmit}>
                    <div className={styles.inputGroup}>
                        <input
                            type="email"
                            placeholder="Email Address"
                            className={styles.input}
                            value={email}
                            onChange={(e) => setEmail(e.target.value)}
                            required
                        />
                    </div>
                    <div className={styles.inputGroup}>
                        <input
                            type="password"
                            placeholder="Password"
                            className={styles.input}
                            value={password}
                            onChange={(e) => setPassword(e.target.value)}
                            required
                        />
                    </div>
                    <button type="submit" className={`${styles.submitButton} button button--primary`} disabled={loading}>
                        {loading ? 'Processing...' : (isLogin ? 'Log In' : 'Sign Up')}
                    </button>
                </form>

                <p className={styles.toggleText}>
                    {isLogin ? "Don't have an account?" : "Already have an account?"}
                    <span className={styles.toggleLink} onClick={() => setIsLogin(!isLogin)}>
                        {isLogin ? 'Sign Up' : 'Log In'}
                    </span>
                </p>
            </div>
        </div>
    );
};
