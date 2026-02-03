/**
 * ChatMessage component for displaying individual chat messages.
 */

import React from 'react';
import type { ChatMessageProps } from './types';
import styles from './ChatBot.module.css';

export function ChatMessage({ message, onCitationClick }: ChatMessageProps): JSX.Element {
  const isUser = message.role === 'user';

  return (
    <div
      className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage} ${
        message.isLoading ? styles.loadingMessage : ''
      }`}
    >
      {message.isLoading ? (
        <p className={styles.messageContent}>
          Thinking<span className={styles.loadingDots}></span>
        </p>
      ) : (
        <>
          <p className={styles.messageContent}>{message.content}</p>
          {message.error && <p className={styles.errorMessage}>{message.error}</p>}
          {message.citations && message.citations.length > 0 && (
            <div className={styles.citations}>
              <div className={styles.citationsLabel}>Sources:</div>
              <div className={styles.citationsList}>
                {message.citations.map((citation, index) => (
                  <span
                    key={citation.chunk_id}
                    className={styles.citationBadge}
                    onClick={() => onCitationClick?.(citation)}
                    title={`${citation.chapter}${citation.section ? ` - ${citation.section}` : ''}`}
                  >
                    [{index + 1}] {citation.chapter.slice(0, 20)}
                    {citation.chapter.length > 20 ? '...' : ''}
                  </span>
                ))}
              </div>
            </div>
          )}
        </>
      )}
    </div>
  );
}
