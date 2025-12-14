import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'book-intro',
    {
      type: 'category',
      label: 'Module 1: ROS Fundamentals',
      items: [
        'module-1-ros-fundamentals',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Basics',
      items: [
        'module-2-simulation-basics',
        'module-2-chapter-1-gazebo-intro',
        'module-2-chapter-2-robot-models',
        'module-2-chapter-3-simulation-environments',
        'module-2-chapter-4-basic-robot-control',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Advanced Simulation',
      items: [
        'module-3-advanced-simulation',
        'module-3-chapter-1-unity-intro',
        'module-3-chapter-2-isaac-sim-basics',
        'module-3-chapter-3-ros-unity-integration',
        'module-3-chapter-4-advanced-isaac-sim',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA and Planning',
      items: [
        'module-4-vla-and-planning',
        'module-4-chapter-1-vla-introduction',
        'module-4-chapter-2-language-to-action',
        'module-4-chapter-3-vision-for-planning',
        'module-4-chapter-4-integrated-vla',
      ],
    },
    'capstone-project',
    {
      type: 'category',
      label: 'Capstone Project Chapters',
      items: [
        'capstone-chapter-1-project-overview',
        'capstone-chapter-2-hardware-software-integration',
        'capstone-chapter-3-vla-implementation',
        'capstone-chapter-4-testing-deployment',
      ],
    },
    'glossary',
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
