/*
 * Blocks.java
 * RVO2 Library Java
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/RVO2/>
 */

package edu.unc.cs.gamma.rvo.examples;

import edu.unc.cs.gamma.rvo.Simulator;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.FastMath;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

// Example showing a demo with 100 agents split in four groups initially
// positioned in four corners of the environment. Each agent attempts to move to
// other side of the environment through a narrow passage generated by four
// obstacles. There is no road map to guide the agents around the obstacles.
class Blocks {
    // Store the goals of the agents.
    private final List<Vector2D> goals = new ArrayList<>();

    // Random number generator.
    private final Random random = new Random();

    private void setupScenario() {
        // Specify the global time step of the simulation.
        Simulator.instance.setTimeStep(0.25);

        // Specify the default parameters for agents that are subsequently
        // added.
        Simulator.instance.setAgentDefaults(15.0, 10, 5.0, 5.0, 2.0, 2.0, Vector2D.ZERO);

        // Add agents, specifying their start position, and store their goals on
        // the opposite side of the environment.
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                Simulator.instance.addAgent(new Vector2D(55.0 + i * 10.0, 55.0 + j * 10.0));
                goals.add(new Vector2D(-75.0, -75.0));

                Simulator.instance.addAgent(new Vector2D(-55.0 - i * 10.0, 55.0 + j * 10.0));
                goals.add(new Vector2D(75.0, -75.0));

                Simulator.instance.addAgent(new Vector2D(55.0 + i * 10.0, -55.0 - j * 10.0));
                goals.add(new Vector2D(-75.0, 75.0));

                Simulator.instance.addAgent(new Vector2D(-55.0 - i * 10.0, -55.0 - j * 10.0));
                goals.add(new Vector2D(75.0, 75.0));
            }
        }

        // Add (polygonal) obstacles, specifying their vertices in
        // counterclockwise order.
        final List<Vector2D> obstacle1 = new ArrayList<>();
        obstacle1.add(new Vector2D(-10.0, 40.0));
        obstacle1.add(new Vector2D(-40.0, 40.0));
        obstacle1.add(new Vector2D(-40.0, 10.0));
        obstacle1.add(new Vector2D(-10.0, 10.0));
        Simulator.instance.addObstacle(obstacle1);

        final List<Vector2D> obstacle2 = new ArrayList<>();
        obstacle2.add(new Vector2D(10.0, 40.0));
        obstacle2.add(new Vector2D(10.0, 10.0));
        obstacle2.add(new Vector2D(40.0, 10.0));
        obstacle2.add(new Vector2D(40.0, 40.0));
        Simulator.instance.addObstacle(obstacle2);

        final List<Vector2D> obstacle3 = new ArrayList<>();
        obstacle3.add(new Vector2D(10.0, -40.0));
        obstacle3.add(new Vector2D(40.0, -40.0));
        obstacle3.add(new Vector2D(40.0, -10.0));
        obstacle3.add(new Vector2D(10.0, -10.0));
        Simulator.instance.addObstacle(obstacle3);

        final List<Vector2D> obstacle4 = new ArrayList<>();
        obstacle4.add(new Vector2D(-10.0, -40.0));
        obstacle4.add(new Vector2D(-10.0, -10.0));
        obstacle4.add(new Vector2D(-40.0, -10.0));
        obstacle4.add(new Vector2D(-40.0, -40.0));
        Simulator.instance.addObstacle(obstacle4);

        // Process the obstacles so that they are accounted for in the
        // simulation.
        Simulator.instance.processObstacles();
    }

    private void updateVisualization() {
        // Output the current global time.
        System.out.print(Simulator.instance.getGlobalTime());

        // Output the current position of all the agents.
        for (int agentNo = 0; agentNo < Simulator.instance.getNumAgents(); agentNo++) {
            System.out.print(" " + Simulator.instance.getAgentPosition(agentNo));
        }

        System.out.println();
    }

    private void setPreferredVelocities() {
        // Set the preferred velocity to be a vector of unit magnitude (speed)
        // in the direction of the goal.
        for (int agentNo = 0; agentNo < Simulator.instance.getNumAgents(); agentNo++) {
            Vector2D goalVector = goals.get(agentNo).subtract(Simulator.instance.getAgentPosition(agentNo));
            final double lengthSq = goalVector.getNormSq();

            if (lengthSq > 1.0) {
                goalVector = goalVector.scalarMultiply(1.0 / FastMath.sqrt(lengthSq));
            }

            Simulator.instance.setAgentPreferredVelocity(agentNo, goalVector);

            // Perturb a little to avoid deadlocks due to perfect symmetry.
            final double angle = random.nextDouble() * 2.0 * FastMath.PI;
            final double distance = random.nextDouble() * 0.0001;

            Simulator.instance.setAgentPreferredVelocity(agentNo, Simulator.instance.getAgentPreferredVelocity(agentNo).add(new Vector2D(FastMath.cos(angle), FastMath.sin(angle)).scalarMultiply(distance)));
        }
    }

    private boolean reachedGoal() {
        // Check if all agents have reached their goals.
        for (int agentNo = 0; agentNo < Simulator.instance.getNumAgents(); agentNo++) {
            if (Simulator.instance.getAgentPosition(agentNo).distanceSq(goals.get(agentNo)) > 400.0) {
                return false;
            }
        }

        return true;
    }

    public static void main(String[] args) {
        final Blocks blocks = new Blocks();

        // Set up the scenario.
        blocks.setupScenario();

        // Perform (and manipulate) the simulation.
        do {
            blocks.updateVisualization();
            blocks.setPreferredVelocities();
            Simulator.instance.doStep();
        }
        while (!blocks.reachedGoal());
    }
}
