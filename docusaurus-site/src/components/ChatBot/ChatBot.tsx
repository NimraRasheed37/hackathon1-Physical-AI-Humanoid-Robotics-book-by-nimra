/**
 * ChatBot component - main chat widget for the textbook.
 */

import React, { useRef, useEffect, useCallback } from 'react';
import type { ChatBotProps } from './types';
import type { Citation } from '../../services/api';
import { config } from '../../services/config';
import { useChatBot } from './useChatBot';
import { ChatMessage } from './ChatMessage';
import { ChatInput } from './ChatInput';
import { TextSelectionPopup } from './TextSelectionPopup';
import styles from './ChatBot.module.css';

export function ChatBot({
  defaultOpen = false,
  onToggle,
  className = '',
}: ChatBotProps): JSX.Element {
  const { state, sendMessage, toggleOpen, setOpen } = useChatBot(defaultOpen);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [state.messages]);

  // Notify parent of toggle
  useEffect(() => {
    onToggle?.(state.isOpen);
  }, [state.isOpen, onToggle]);

  const handleCitationClick = (citation: Citation) => {
    // TODO: Implement navigation to citation source
    console.log('Citation clicked:', citation);
  };

  const handleSubmit = (query: string) => {
    sendMessage(query);
  };

  // Handle "Explain" action from text selection
  const handleExplainSelection = useCallback(
    (text: string) => {
      setOpen(true);
      sendMessage(`Please explain the following text from the textbook: "${text}"`, { text });
    },
    [sendMessage, setOpen]
  );

  // Handle "Ask about" action from text selection
  const handleAskAboutSelection = useCallback(
    (text: string) => {
      setOpen(true);
      // Open chat with the selected text as context, user can type their question
      sendMessage(`I have a question about this text: "${text.slice(0, 200)}${text.length > 200 ? '...' : ''}"`, { text });
    },
    [sendMessage, setOpen]
  );

  return (
    <div className={`${styles.chatBotContainer} ${className}`}>
      {/* Text Selection Popup */}
      <TextSelectionPopup
        onExplain={handleExplainSelection}
        onAsk={handleAskAboutSelection}
      />
      {/* Chat Window */}
      {state.isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.header}>
            <div>
              <h3 className={styles.headerTitle}>{config.chatBotTitle}</h3>
              <p className={styles.headerSubtitle}>{config.chatBotSubtitle}</p>
            </div>
            <button
              className={styles.closeButton}
              onClick={toggleOpen}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {state.messages.length === 0 ? (
              <div className={styles.emptyState}>
                <span className={styles.emptyStateIcon}>💬</span>
                <p className={styles.emptyStateText}>
                  Hi! I'm your AI assistant for this textbook.
                  <br />
                  Ask me anything about Physical AI and Humanoid Robotics!
                </p>
              </div>
            ) : (
              <>
                {state.messages.map((message) => (
                  <ChatMessage
                    key={message.id}
                    message={message}
                    onCitationClick={handleCitationClick}
                  />
                ))}
                <div ref={messagesEndRef} />
              </>
            )}
          </div>

          {/* Input */}
          <ChatInput onSubmit={handleSubmit} disabled={state.isLoading} />
        </div>
      )}

      {/* Toggle Button */}
      <button
        className={styles.toggleButton}
        onClick={toggleOpen}
        aria-label={state.isOpen ? 'Close chat' : 'Open chat'}
        aria-expanded={state.isOpen}
      >
        <span className={styles.toggleIcon}>{state.isOpen ? '×' : '💬'}</span>
      </button>
    </div>
  );
}

export default ChatBot;
