
# Implementing the Particle Filter

The only file you should modify is particle_filter.cpp


# Data





# Rubrics

- Does your particle filter localize the vehicle to within the desired accuracy?

  This criteria is checked automatically when you do ./run.sh in the
  terminal. If the output says "Success! Your particle filter passed!"
  then it means you’ve met this criteria.


- Does your particle run within the specified time of 100 seconds?

  This criteria is checked automatically when you do ./run.sh in the
  terminal. If the output says "Success! Your particle filter passed!"
  then it means you’ve met this criteria.


- Does your code use a particle filter to localize the robot?

  There may be ways to “beat” the automatic grader without actually
  implementing the full particle filter. You will meet this criteria
  if the methods you write in particle_filter.cpp behave as expected.



# Markov Localization and the Kidnapped Vehicle Project

The localization module culminates in the Kidnapped Vehicle
Project. In that project our vehicle has been kidnapped and placed in
an unknown location. We must leverage our knowledge of localization to
determine where our vehicle is. The Kidnapped Vehicle Project relies
heavily on the particle filter approach to localization, particularly
"Implementation of a Particle Filter," leaving the question; How does
Markov Localization relate to the Kidnapped Vehicle project?

Markov Localization or Bayes Filter for Localization is a generalized
filter for localization and all other localization approaches are
realizations of this approach, as discussed here. By learning how to
derive and implement (coding exercises) this filter we develop
intuition and methods that will help us solve any vehicle localization
task, including implementation of a particle filter. Generally we can
think of our vehicle location as a distribution, each time we move our
distribution becomes more diffuse (wider). We pass our variables (map
data, observation data, and control data) into the filter to
concentrate (narrow) this distribution, at each time step. Each state
prior to applying the filter represents our prior and the narrowed
distribution represents our Bayes posterior.

Beyond a deep understanding of how Bayesian methods are applied to
localization there are some specific instances that tie directly to
the Kidnapped Vehicle Project, which include:

- Localization Posterior and Explanation: relates to project in terms
  of searching for observations within sensor range.

- Coding Exercise 1 (Code Structure of the Input Data): Here we learn
  how to access our observation, control, and map variables, learn the
  structure of our data, and build the C++ scaffolding for
  initializing a filter.

- Coding Exercises 2 and 3: In these exercises we learn how to
  implement a Bayesian Localization Filter by incorporating our
  variables at each time step, into the filter.


# Formal Definition of Variables

z1:t
​​  represents the observation vector from time 0 to t (range measurements, bearing, images, etc.).

u1:t
​​  represents the control vector from time 0 to t (yaw/pitch/roll rates and velocities).

m represents the map (grid maps, feature maps, landmarks)

xt
​​  represents the pose (position (x,y) + orientation θ)


# Apply Bayes Rule with Additional Conditions

