# Markov Localization and the Kidnapped Vehicle Project

The localization module culminates in the Kidnapped Vehicle Project. In that project our vehicle has been kidnapped and placed in an unknown location. We must leverage our knowledge of localization to determine where our vehicle is. The Kidnapped Vehicle Project relies heavily on the particle filter approach to localization, particularly "Implementation of a Particle Filter," leaving the question; How does Markov Localization relate to the Kidnapped Vehicle project?

Markov Localization or Bayes Filter for Localization is a generalized filter for localization and all other localization approaches are realizations of this approach, as discussed here. By learning how to derive and implement (coding exercises) this filter we develop intuition and methods that will help us solve any vehicle localization task, including implementation of a particle filter. Generally we can think of our vehicle location as a distribution, each time we move our distribution becomes more diffuse (wider). We pass our variables (map data, observation data, and control data) into the filter to concentrate (narrow) this distribution, at each time step. Each state prior to applying the filter represents our prior and the narrowed distribution represents our Bayes posterior.

Beyond a deep understanding of how Bayesian methods are applied to localization there are some specific instances that tie directly to the Kidnapped Vehicle Project, which include:

- Localization Posterior and Explanation: relates to project in terms of searching for observations within sensor range.
- Coding Exercise 1 (Code Structure of the Input Data): Here we learn how to access our observation, control, and map variables, learn the structure of our data, and build the C++ scaffolding for initializing a filter.
- Coding Exercises 2 and 3: In these exercises we learn how to implement a Bayesian Localization Filter by incorporating our variables at each time step, into the filter.
