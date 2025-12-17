import React from 'react';
import ChatBot from '@site/src/components/ChatBot';
import TextSelectionHandler from '@site/src/components/TextSelection';

export default function Root({ children }) {
    return (
        <>
            {children}
            <TextSelectionHandler />
            <ChatBot />
        </>
    );
}
