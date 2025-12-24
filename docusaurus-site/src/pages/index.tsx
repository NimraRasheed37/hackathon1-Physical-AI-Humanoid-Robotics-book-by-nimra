import React from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import HomepageFeatures from "@site/src/components/HomepageFeatures";

import Heading from "@theme/Heading";
import styles from "./index.module.css";

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext(); // Keep siteConfig for potential future use or consistency
  const bookTitle = "Physical AI & Humanoid Robotics";
  const bookSubtitle = "AI-driven humanoid robotics using ROS 2, Gazebo, Isaac, and VLA";
  return (
    <header className={clsx("hero hero--primary", styles.heroBanner)}>
      <div className="container">
        {/* <img className={styles.heroImg} src="/AI-textbook/img/undraw_docusaurus_react.svg" alt="Robot themed imagery" /> */}
        <Heading as="h1" className="hero__title">
          {bookTitle}
        </Heading>
        <p className="hero__subtitle">{bookSubtitle}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/module-1-ros2/chapter-1-ros2-fundamentals"
          >
            Start Reading 📖
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const bookTitle = "Physical AI & Humanoid Robotics"; // Use the new title for Layout as well
  return (
    <Layout
      title={`Home | ${bookTitle}`}
      description="Learn about AI-driven humanoid robotics using ROS 2, Gazebo, Isaac, and VLA"
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
