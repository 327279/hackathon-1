import React, { useState, useEffect } from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import styles from './styles.module.css';
import { TranslateButton } from '@site/src/components/TranslateButton';
import { PersonalizeButton } from '@site/src/components/PersonalizeButton';
import { AuthModal } from '@site/src/components/Auth';
import { OnboardingModal } from '@site/src/components/Onboarding';
import ReactMarkdown from 'react-markdown'; // We might need to install this, or use simple rendering

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
    const [customContent, setCustomContent] = useState<string | null>(null);
    const [articleContent, setArticleContent] = useState("");
    const [showAuth, setShowAuth] = useState(false);
    const [showOnboarding, setShowOnboarding] = useState(false);

    // Metadata from props (Docusaurus specific)
    const { frontMatter, metadata } = (props as any).content || {};
    const title = metadata?.title || "Chapter";

    useEffect(() => {
        const article = document.querySelector('article');
        if (article) {
            setArticleContent(article.innerText);
        }
    }, []);

    return (
        <>
            <div className={styles.toolbar} style={{ marginBottom: '1rem', display: 'flex', justifyContent: 'flex-end', gap: '0.5rem' }}>
                <PersonalizeButton
                    content={articleContent}
                    title={title}
                    onPersonalized={setCustomContent}
                />
                <TranslateButton
                    content={articleContent}
                    onTranslated={setCustomContent}
                />
                <button
                    className="button button--secondary"
                    style={{ marginLeft: '1rem' }}
                    onClick={() => setShowAuth(true)}
                >
                    Login
                </button>
            </div>

            {customContent ? (
                <div className="markdown" style={{ padding: '2rem', background: 'rgba(255,255,255,0.05)', borderRadius: '8px' }}>
                    <h3>✨ AI Adapted Content</h3>
                    <div style={{ whiteSpace: 'pre-wrap' }}>{customContent}</div>
                    <button
                        className="button button--link"
                        onClick={() => setCustomContent(null)}
                        style={{ marginTop: '1rem' }}
                    >
                        Revert to Original
                    </button>
                </div>
            ) : (
                <Content {...props} />
            )}

            <AuthModal isOpen={showAuth} onClose={() => { setShowAuth(false); setShowOnboarding(true); }} />
            <OnboardingModal isOpen={showOnboarding} onClose={() => setShowOnboarding(false)} />
        </>
    );
}

