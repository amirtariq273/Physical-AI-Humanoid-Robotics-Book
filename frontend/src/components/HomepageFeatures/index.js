import React from 'react';
import clsx from 'clsx';
import PropTypes from 'prop-types';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Modular Learning',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Explore a comprehensive curriculum designed to take you from the basics
        of ROS 2 to advanced AI-driven robotics.
      </>
    ),
  },
  {
    title: 'Hands-On Projects',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Apply your knowledge with practical, hands-on projects that simulate
        real-world robotics challenges.
      </>
    ),
  },
  {
    title: 'Powered by Cutting-Edge Tech',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Leverage the power of Docusaurus, React, and other modern technologies
        to build a robust learning platform.
      </>
    ),
  },
];

function Feature({ Svg, title, description }) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

Feature.propTypes = {
  Svg: PropTypes.elementType.isRequired,
  title: PropTypes.string.isRequired,
  description: PropTypes.node.isRequired,
};

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}