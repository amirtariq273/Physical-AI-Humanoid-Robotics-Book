import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';

// ہیڈر کا ڈیزائن
function HomepageHeader() {
  return (
    <header className={clsx('hero', 'heroBanner')}>
      <div className="container">
        {/* Main Title */}
        <h1 className="hero__title">Physical AI & Humanoid Robotics</h1>
        {/* Subtitle */}
        <p className="hero__subtitle">
          Mastering Spec-Kit Plus: From Concept to Implementation
        </p>
        <div className="buttons" style={{ marginTop: '20px' }}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro"
          >
            📘 Start Reading Modules
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  return (
    <Layout
      description="Physical AI Hackathon Project generated via Spec-Kit">
      <HomepageHeader />
      <main>
        <div className="container" style={{ padding: '50px 0' }}>
          <div className="row">
            {/* Box 1 */}
            <div className="col col--4">
              <div className="text--center">
                <div style={{ fontSize: '3rem' }}>🚀</div>
                <h3>Spec-Kit Driven</h3>
                <p>
                  Built using the advanced <b>Spec-Kit framework</b> to ensure
                  structured, verified, and high-quality technical content
                  generation.
                </p>
              </div>
            </div>
            {/* Box 2 */}
            <div className="col col--4">
              <div className="text--center">
                <div style={{ fontSize: '3rem' }}>🤖</div>
                <h3>Gemini CLI Powered</h3>
                <p>
                  Leveraging <b>Google Gemini CLI</b> to synthesize complex
                  Physical AI concepts into easy-to-understand modules.
                </p>
              </div>
            </div>
            {/* Box 3 */}
            <div className="col col--4">
              <div className="text--center">
                <div style={{ fontSize: '3rem' }}>🏆</div>
                <h3>Hackathon Project</h3>
                <p>
                  Developed for the{' '}
                  <b>Physical AI & Humanoid Robotics Hackathon</b>. Covering
                  Chapter 14: Master Spec-Kit Plus.
                </p>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
