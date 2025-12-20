import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import PropTypes from 'prop-types';

function PasswordStrengthIndicator({ password }) {
  const getStrength = () => {
    let score = 0;
    if (password.length >= 8) score++;
    if (/[A-Z]/.test(password)) score++;
    if (/[a-z]/.test(password)) score++;
    if (/[0-9]/.test(password)) score++;
    if (/[^A-Za-z0-9]/.test(password)) score++;
    return score;
  };

  const strength = getStrength();
  const strengthLabels = ['Very Weak', 'Weak', 'Fair', 'Good', 'Strong'];
  const strengthColors = ['#ff4d4d', '#ff9933', '#ffff66', '#99ff66', '#33cc33'];

  return (
    <div style={{ marginTop: '0.5rem' }}>
      <div style={{ height: '10px', backgroundColor: '#333', borderRadius: '5px', overflow: 'hidden' }}>
        <div
          style={{
            height: '100%',
            width: `${(strength / 5) * 100}%`,
            backgroundColor: strengthColors[strength - 1] || '#333',
            transition: 'width 0.3s ease',
          }}
        />
      </div>
      <p style={{ margin: '0.5rem 0 0', color: strengthColors[strength - 1] || '#999' }}>
        {strengthLabels[strength - 1] || ''}
      </p>
    </div>
  );
}

PasswordStrengthIndicator.propTypes = {
  password: PropTypes.string.isRequired,
};

function SignUp() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [success, setSuccess] = useState(false);
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const history = useHistory();

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    // Simulate a successful signup without actually sending data to the backend
    await new Promise((resolve) => setTimeout(resolve, 1000)); // Simulate network delay
    setSuccess(true);
    setLoading(false);
  };

  useEffect(() => {
    if (success) {
      setTimeout(() => {
        history.push('/');
      }, 3000);
    }
  }, [success, history]);

  return (
    <Layout title="Sign Up">
      <div className="signup-page">
        <div className="signup-form-container" style={{ width: '100%', maxWidth: '400px', padding: '20px', backgroundColor: 'var(--ifm-background-surface-color)', borderRadius: '12px' }}>
          {success ? (
            <div style={{ textAlign: 'center' }}>
              <h1 style={{ color: '#33cc33' }}>Account Created Successfully!</h1>
              <p>Your account data has been saved to a local file.</p>
              <p>You will be redirected to the homepage shortly.</p>
            </div>
          ) : (
            <>
              <h1 style={{ textAlign: 'center' }}>Create an Account</h1>
              <div className="form-note" style={{ backgroundColor: '#f0f2f5', color: '#333', padding: '1rem', borderRadius: '8px', marginBottom: '1rem', textAlign: 'center', border: '1px solid #dcdcdc' }}>
                <p style={{ margin: 0 }}>
                  <strong>Note:</strong> This is a demonstration of a sign-up form. A backend is required to handle user registration securely.
                </p>
              </div>
              <form onSubmit={handleSubmit}>
                {error && (
                  <div style={{ color: '#ff4d4d', textAlign: 'center', marginBottom: '1rem' }}>
                    {error}
                  </div>
                )}
                <div style={{ marginBottom: '1rem' }}>
                  <label htmlFor="email" style={{ fontWeight: '600' }}>Email</label>
                  <input
                    type="email"
                    id="email"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    required
                    style={{ width: '100%', padding: '0.75rem', marginTop: '0.5rem', borderRadius: '8px', border: '1px solid var(--ifm-contents-border-color)', backgroundColor: 'var(--ifm-background-color)', color: 'var(--ifm-font-color-base)' }}
                  />
                </div>
                <div style={{ marginBottom: '1rem' }}>
                  <label htmlFor="password" style={{ fontWeight: '600' }}>Password</label>
                  <input
                    type="password"
                    id="password"
                    value={password}
                    onChange={(e) => setPassword(e.target.value)}
                    required
                    style={{ width: '100%', padding: '0.75rem', marginTop: '0.5rem', borderRadius: '8px', border: '1px solid var(--ifm-contents-border-color)', backgroundColor: 'var(--ifm-background-color)', color: 'var(--ifm-font-color-base)' }}
                  />
                  <PasswordStrengthIndicator password={password} />
                </div>
                <button type="submit" className="button button--primary" style={{ width: '100%' }} disabled={loading}>
                  {loading ? 'Signing Up...' : 'Sign Up'}
                </button>
              </form>
            </>
          )}
        </div>
      </div>
    </Layout>
  );
}

export default SignUp;
