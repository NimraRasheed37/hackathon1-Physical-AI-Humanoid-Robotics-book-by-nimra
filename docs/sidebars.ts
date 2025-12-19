import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'introduction/intro',
        'introduction/book-intro',
        'introduction/glossary',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS Fundamentals',
      items: [
        'module-1-ros-fundamentals/module-1-ros-fundamentals',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Basics',
      items: [
        'module-2-simulation-basics/module-2-simulation-basics',
        'module-2-simulation-basics/module-2-chapter-1-introduction-to-gazebo',
        'module-2-simulation-basics/module-2-chapter-2-robot-models',
        'module-2-simulation-basics/module-2-chapter-3-simulation-environments',
        'module-2-simulation-basics/module-2-chapter-4-basic-robot-control',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Advanced Simulation',
      items: [
        'module-3-advanced-simulation/module-3-advanced-simulation',
        'module-3-advanced-simulation/module-3-chapter-1-unity-intro',
        'module-3-advanced-simulation/module-3-chapter-2-isaac-sim-basics',
        'module-3-advanced-simulation/module-3-chapter-3-ros-unity-integration',
        'module-3-advanced-simulation/module-3-chapter-4-advanced-isaac-sim',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA and Planning',
      items: [
        'module-4-vla-and-planning/module-4-vla-and-planning',
        'module-4-vla-and-planning/module-4-chapter-1-vla-introduction',
        'module-4-vla-and-planning/module-4-chapter-2-language-to-action',
        'module-4-vla-and-planning/module-4-chapter-3-vision-for-planning',
        'module-4-vla-and-planning/module-4-chapter-4-integrated-vla',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project Chapters',
      items: [
        'capstone-project/capstone-project',
        'capstone-project/capstone-chapter-1-project-overview',
        'capstone-project/capstone-chapter-2-hardware-software-integration',
        'capstone-project/capstone-chapter-3-vla-implementation',
        'capstone-project/capstone-chapter-4-testing-deployment',
      ],
    },
  ],
};

export default sidebars;