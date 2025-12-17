import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)} style={{ padding: '4rem 0', background: 'transparent' }}>
      <div className="container">
        <div className="row">
          <div className="col col--8 col--offset-2 text--center animate-fade-in">
            <h1 className="hero__title text-gradient" style={{ fontSize: '4rem', fontWeight: 800, marginBottom: '1rem' }}>
              {siteConfig.title}
            </h1>
            <p className="hero__subtitle animate-delay-100" style={{ fontSize: '1.5rem', opacity: 0.9 }}>
              {siteConfig.tagline}
            </p>
            <div className={clsx(styles.buttons, 'animate-delay-200')} style={{ marginTop: '2rem' }}>
              <Link
                className="button button--primary button--lg"
                style={{ padding: '1rem 2rem', fontSize: '1.2rem' }}
                to="/docs/intro">
                Start Learning Now →
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function Feature({ title, description, icon, link, delay }) {
  return (
    <div className={clsx('col col--4 animate-fade-in', delay)} style={{ marginBottom: '2rem' }}>
      <Link to={link} style={{ textDecoration: 'none', color: 'inherit' }}>
        <div className="glass-panel" style={{ padding: '2rem', height: '100%', transition: 'transform 0.3s ease', cursor: 'pointer' }}>
          <div style={{ fontSize: '3rem', marginBottom: '1rem' }}>{icon}</div>
          <Heading as="h3" style={{ marginBottom: '0.5rem' }}>{title}</Heading>
          <p style={{ opacity: 0.8 }}>{description}</p>
        </div>
      </Link>
    </div>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Master AI Robotics`}
      description="The ultimate guide to Physical AI, Humanoid Robotics, and ROS 2">

      {/* Background Decorator */}
      <div style={{
        position: 'fixed',
        top: 0,
        left: 0,
        width: '100vw',
        height: '100vh',
        zIndex: -1,
        background: `radial-gradient(circle at 50% 50%, rgba(124, 58, 237, 0.15) 0%, rgba(3, 7, 18, 0) 50%)`,
        pointerEvents: 'none'
      }} />

      <HomepageHeader />

      <main>
        <div className="container" style={{ padding: '4rem 0' }}>
          <div className="row">
            <Feature
              title="Module 1: ROS 2"
              icon="🤖"
              description="Master the Robot Operating System 2, nodes, topics, services, and URDF modeling."
              link="/docs/module-1-ros2"
              delay="animate-delay-100"
            />
            <Feature
              title="Module 2: Simulation"
              icon="🎮"
              description="Learn simulation-first development with Gazebo, Unity, and physics engines."
              link="/docs/module-2-simulation"
              delay="animate-delay-200"
            />
            <Feature
              title="Module 3: Isaac Platform"
              icon="⚡"
              description="Leverage NVIDIA Isaac Sim, Isaac ROS, and GPU-accelerated perception."
              link="/docs/module-3-isaac"
              delay="animate-delay-300"
            />
            <Feature
              title="Module 4: VLA & Humanoids"
              icon="🧠"
              description="Build Vision-Language-Action models and deploy to humanoid robots."
              link="/docs/module-4-vla"
              delay="animate-delay-100" // Wrap around delay
            />
          </div>
        </div>
      </main>
    </Layout>
  );
}

