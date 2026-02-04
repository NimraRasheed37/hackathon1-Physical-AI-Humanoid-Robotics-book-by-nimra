/**
 * Root theme wrapper that adds the ChatBot to all pages.
 *
 * This component wraps the entire Docusaurus application and
 * injects the ChatBot widget globally.
 */

import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

interface RootProps {
  children: React.ReactNode;
}

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  isLoading?: boolean;
}

// Modern gradient theme colors
const theme = {
  primary: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
  primarySolid: '#667eea',
  secondary: 'linear-gradient(135deg, #f093fb 0%, #f5576c 100%)',
  accent: '#764ba2',
  userBubble: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
  assistantBubble: '#f8f9fc',
  background: '#ffffff',
  headerBg: 'linear-gradient(135deg, #1a1a2e 0%, #16213e 100%)',
  text: '#1a1a2e',
  textLight: '#6b7280',
  border: '#e5e7eb',
  shadow: '0 25px 50px -12px rgba(0, 0, 0, 0.25)',
};

function ChatWidget() {
  const { siteConfig } = useDocusaurusContext();
  const apiUrl = (siteConfig.customFields?.apiBaseUrl as string) || 'http://localhost:8001';

  const [isOpen, setIsOpen] = useState(false);
  const [mounted, setMounted] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    setMounted(true);
  }, []);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const sendMessage = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: input.trim(),
    };

    const assistantId = (Date.now() + 1).toString();
    const loadingMessage: Message = {
      id: assistantId,
      role: 'assistant',
      content: '',
      isLoading: true,
    };

    setMessages(prev => [...prev, userMessage, loadingMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch(`${apiUrl}/api/v1/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: userMessage.content }),
      });

      const data = await response.json();

      setMessages(prev =>
        prev.map(msg =>
          msg.id === assistantId
            ? { ...msg, content: data.answer || data.detail?.message || 'Error occurred', isLoading: false }
            : msg
        )
      );
    } catch (error) {
      setMessages(prev =>
        prev.map(msg =>
          msg.id === assistantId
            ? { ...msg, content: 'Failed to connect to server', isLoading: false }
            : msg
        )
      );
    } finally {
      setIsLoading(false);
    }
  };

  if (!mounted) return null;

  return (
    <div style={{ position: 'fixed', bottom: '24px', right: '24px', zIndex: 1000 }}>
      {/* Floating Action Button */}
      <button
        onClick={() => setIsOpen(!isOpen)}
        style={{
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          border: 'none',
          background: theme.primary,
          color: 'white',
          cursor: 'pointer',
          fontSize: '26px',
          boxShadow: '0 8px 25px rgba(102, 126, 234, 0.4)',
          transition: 'all 0.3s ease',
          transform: isOpen ? 'rotate(90deg)' : 'rotate(0deg)',
        }}
        onMouseOver={e => {
          e.currentTarget.style.transform = isOpen ? 'rotate(90deg) scale(1.1)' : 'scale(1.1)';
          e.currentTarget.style.boxShadow = '0 12px 35px rgba(102, 126, 234, 0.5)';
        }}
        onMouseOut={e => {
          e.currentTarget.style.transform = isOpen ? 'rotate(90deg)' : 'rotate(0deg)';
          e.currentTarget.style.boxShadow = '0 8px 25px rgba(102, 126, 234, 0.4)';
        }}
      >
        {isOpen ? '✕' : '🤖'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div style={{
          position: 'absolute',
          bottom: '75px',
          right: '0',
          width: '400px',
          height: '550px',
          background: theme.background,
          borderRadius: '20px',
          boxShadow: theme.shadow,
          display: 'flex',
          flexDirection: 'column',
          overflow: 'hidden',
          animation: 'slideUp 0.3s ease',
        }}>
          {/* Header */}
          <div style={{
            padding: '20px 24px',
            background: theme.headerBg,
            color: 'white',
            display: 'flex',
            alignItems: 'center',
            gap: '14px',
          }}>
            <div style={{
              width: '45px',
              height: '45px',
              borderRadius: '12px',
              background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              fontSize: '22px',
            }}>
              🤖
            </div>
            <div>
              <div style={{ fontWeight: '600', fontSize: '16px', letterSpacing: '0.3px' }}>
                AI Study Assistant
              </div>
              <div style={{ fontSize: '12px', opacity: 0.8, marginTop: '2px' }}>
                Physical AI & Humanoid Robotics
              </div>
            </div>
            <button
              onClick={() => setIsOpen(false)}
              style={{
                marginLeft: 'auto',
                background: 'rgba(255,255,255,0.1)',
                border: 'none',
                color: 'white',
                width: '32px',
                height: '32px',
                borderRadius: '8px',
                cursor: 'pointer',
                fontSize: '16px',
              }}
            >
              ✕
            </button>
          </div>

          {/* Messages */}
          <div style={{
            flex: 1,
            padding: '20px',
            overflowY: 'auto',
            background: 'linear-gradient(180deg, #f8f9fc 0%, #ffffff 100%)',
          }}>
            {messages.length === 0 ? (
              <div style={{ textAlign: 'center', marginTop: '60px' }}>
                <div style={{ fontSize: '50px', marginBottom: '16px' }}>📚</div>
                <div style={{ color: theme.text, fontWeight: '600', fontSize: '18px', marginBottom: '8px' }}>
                  Welcome!
                </div>
                <div style={{ color: theme.textLight, fontSize: '14px', lineHeight: '1.6' }}>
                  I can help you understand concepts from<br/>the Physical AI & Humanoid Robotics textbook.
                </div>
                <div style={{
                  marginTop: '24px',
                  display: 'flex',
                  flexWrap: 'wrap',
                  gap: '8px',
                  justifyContent: 'center',
                }}>
                  {['What is URDF?', 'Explain ROS2', 'Isaac Sim basics'].map(q => (
                    <button
                      key={q}
                      onClick={() => { setInput(q); }}
                      style={{
                        padding: '8px 16px',
                        background: 'white',
                        border: `1px solid ${theme.border}`,
                        borderRadius: '20px',
                        fontSize: '13px',
                        color: theme.primarySolid,
                        cursor: 'pointer',
                        transition: 'all 0.2s',
                      }}
                      onMouseOver={e => {
                        e.currentTarget.style.background = theme.primarySolid;
                        e.currentTarget.style.color = 'white';
                        e.currentTarget.style.borderColor = theme.primarySolid;
                      }}
                      onMouseOut={e => {
                        e.currentTarget.style.background = 'white';
                        e.currentTarget.style.color = theme.primarySolid;
                        e.currentTarget.style.borderColor = theme.border;
                      }}
                    >
                      {q}
                    </button>
                  ))}
                </div>
              </div>
            ) : (
              messages.map(msg => (
                <div
                  key={msg.id}
                  style={{
                    marginBottom: '16px',
                    display: 'flex',
                    justifyContent: msg.role === 'user' ? 'flex-end' : 'flex-start',
                  }}
                >
                  {msg.role === 'assistant' && (
                    <div style={{
                      width: '32px',
                      height: '32px',
                      borderRadius: '10px',
                      background: theme.primary,
                      display: 'flex',
                      alignItems: 'center',
                      justifyContent: 'center',
                      fontSize: '16px',
                      marginRight: '10px',
                      flexShrink: 0,
                    }}>
                      🤖
                    </div>
                  )}
                  <div style={{
                    maxWidth: '75%',
                    padding: '12px 16px',
                    borderRadius: msg.role === 'user' ? '18px 18px 4px 18px' : '18px 18px 18px 4px',
                    background: msg.role === 'user' ? theme.userBubble : theme.assistantBubble,
                    color: msg.role === 'user' ? 'white' : theme.text,
                    fontSize: '14px',
                    lineHeight: '1.5',
                    boxShadow: msg.role === 'user' ? '0 4px 15px rgba(102, 126, 234, 0.3)' : '0 2px 8px rgba(0,0,0,0.06)',
                  }}>
                    {msg.isLoading ? (
                      <div style={{ display: 'flex', gap: '4px', padding: '4px 0' }}>
                        <span style={{ animation: 'bounce 1s infinite', animationDelay: '0ms' }}>●</span>
                        <span style={{ animation: 'bounce 1s infinite', animationDelay: '150ms' }}>●</span>
                        <span style={{ animation: 'bounce 1s infinite', animationDelay: '300ms' }}>●</span>
                      </div>
                    ) : msg.content}
                  </div>
                </div>
              ))
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div style={{
            padding: '16px 20px',
            borderTop: `1px solid ${theme.border}`,
            background: 'white',
            display: 'flex',
            gap: '12px',
            alignItems: 'center',
          }}>
            <input
              type="text"
              value={input}
              onChange={e => setInput(e.target.value)}
              onKeyDown={e => e.key === 'Enter' && sendMessage()}
              placeholder="Type your question..."
              disabled={isLoading}
              style={{
                flex: 1,
                padding: '14px 20px',
                border: `2px solid ${theme.border}`,
                borderRadius: '25px',
                fontSize: '14px',
                outline: 'none',
                transition: 'border-color 0.2s',
                background: '#f8f9fc',
              }}
              onFocus={e => e.currentTarget.style.borderColor = theme.primarySolid}
              onBlur={e => e.currentTarget.style.borderColor = theme.border}
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !input.trim()}
              style={{
                width: '48px',
                height: '48px',
                borderRadius: '50%',
                border: 'none',
                background: theme.primary,
                color: 'white',
                cursor: isLoading || !input.trim() ? 'not-allowed' : 'pointer',
                opacity: isLoading || !input.trim() ? 0.5 : 1,
                fontSize: '18px',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                boxShadow: '0 4px 15px rgba(102, 126, 234, 0.3)',
                transition: 'all 0.2s',
              }}
              onMouseOver={e => {
                if (!isLoading && input.trim()) {
                  e.currentTarget.style.transform = 'scale(1.05)';
                }
              }}
              onMouseOut={e => {
                e.currentTarget.style.transform = 'scale(1)';
              }}
            >
              ➤
            </button>
          </div>
        </div>
      )}

      {/* Animations */}
      <style>{`
        @keyframes slideUp {
          from { opacity: 0; transform: translateY(20px); }
          to { opacity: 1; transform: translateY(0); }
        }
        @keyframes bounce {
          0%, 60%, 100% { transform: translateY(0); }
          30% { transform: translateY(-4px); }
        }
      `}</style>
    </div>
  );
}

export default function Root({ children }: RootProps): JSX.Element {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
