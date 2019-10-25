/**
 * Implementation of the "room Q-learning example", as described in the following article:
 * http://mnemstudio.org/path-finding-q-learning-tutorial.htm
 * 
 * The algorithm for the training stage goes as follows:
 *  1. Set the gamma parameter, and environment rewards in matrix R.
 *  2. Initialize matrix Q to zero.
 *  3. For each episode:
 *      Select a random initial state.
 *          Do While the goal state hasn't been reached.
 *          Select one among all possible actions for the current state. (randomly)
 *          Using this possible action, consider going to the next state.
 *          Get maximum Q value for this next state based on all possible actions.
 *          Compute: Q(state, action) = R(state, action) + Gamma * Max[Q(next state, all actions)]
 *          Set the next state as the current state.
 *      End Do
 *     End For
 * 
 * Evaluation stage:
 *  1. Set current state = initial state.
 *  2. From current state, find the action with the highest Q value.
 *  3. Set current state = next state.
 *  4. Repeat Steps 2 and 3 until current state = goal state.
 * 
 * R - Matrix of rewards:
 *          -1 reward goes for impossible moves.
 *          0 reward goes for moves which don't directly go to the goal state.
 *          100 reward goes for moves which directly lead to the goal state.
 *   - Rows = states
 *   - Columns = actions
 * 
 * Q - Matrix of qualities. This represents the agent's memory. 
 *   - In the training stage (qLearnTrainEpisode), this matrix gets filled with values.
 *   - In the evaluation stage (qLearnUtilize), the path towards the goal state is determined
 *     based on the values in this matrix.
 * gamma - hyperparameter, constant value, determines how much the previous values in the Q matrix should matter.
 */
'use strict';
const R = [
    [-1, -1, -1, -1, 0, -1],
    [-1, -1, -1, 0, -1, 100],
    [-1, -1, -1, 0, -1, -1],
    [-1, 0, 0, -1, 0, -1],
    [0, -1, -1, 0, -1, 100],
    [-1, 0, -1, -1, 0, 100],
];
let Q = [
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
];
const GAMMA = 0.8;
const GOAL_STATE = 5;
const NUMBER_OF_STATES = 6;
const EPISODES = 100;

// Training phase.
for (let i = 0; i < EPISODES; ++i) {
    qLearnTrainEpisode(Math.floor(Math.random() * NUMBER_OF_STATES));
}
// Prettify the Q matrix (only have 2 decimal places for float numbers).
const QPretty = Q.map(q =>
    q.map( qi => qi.toString().slice(0, (qi.toString().indexOf(".")) + 3))
);
console.log(QPretty);

// Generate all the states [0, number_of_states).
const states = [...Array(NUMBER_OF_STATES)].map((v, i) => i);
// Call the utilize function for every state.
states.forEach(s => qLearnUtilize(s));

/**
 * Training stage
 * Do While the goal state hasn't been reached.
 *   Select one among all possible actions for the current state.
 *   Using this possible action, consider going to the next state.
 *   Get maximum Q value for this next state based on all possible actions.
 *   Compute: Q(state, action) = R(state, action) + Gamma * Max[Q(next state, all actions)]
 *   Set the next state as the current state.
 * End Do
 * 
 * @param {*} initial_state Starting position.
 */
function qLearnTrainEpisode(initial_state) {
    if (initial_state < 0 || initial_state > 5) {
        console.log("Initial state should be between 0 and 5");
        return;
    }
    let state = initial_state;
    do {
        let possibleActions = R[state].map((r, a) => ({reward: r, action: a}));
        // Filter out the "impossible" actions
        possibleActions = possibleActions.filter((p) => p.reward !== -1);
        // Choose a random action. 
        let action = possibleActions[Math.floor(Math.random() * possibleActions.length)].action;
        // Calculate the quality of the move.
        Q[state][action] = R[state][action] + GAMMA * Math.max(...Q[action]);
        // Move to the next action.
        state = action;
    } while (state != GOAL_STATE);
}

/**
 * Evaluation stage:
 *  1. Set current state = initial state.
 *  2. From current state, find the action with the highest Q value.
 *  3. Set current state = next state.
 *  4. Repeat Steps 2 and 3 until current state = goal state.
 * 
 * @param {*} initial_state Starting position.
 */
function qLearnUtilize(initial_state) {
    if (initial_state < 0 || initial_state > 5) {
        console.log("Initial state should be between 0 and 5");
        return;
    }
    let state = initial_state;
    let path = [];
    path.push(state);
    do {
        // Get the index to the maximum value element.
        let nextState = Q[state].reduce((iMax, x, i, arr) => x > arr[iMax] ? i : iMax, 0);
        state = nextState;
        path.push(state);
    } while (state != GOAL_STATE);
    console.log("Path towards the end goal: " + path);
}