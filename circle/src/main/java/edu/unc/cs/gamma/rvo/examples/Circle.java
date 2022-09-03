/*
 * Circle.java
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

// Example showing a demo with 250 agents initially positioned evenly
// distributed on a circle attempting to move to the antipodal position on the
// circle.
class Circle {
    // Store the goals of the agents.
    private final List<Vector2D> goals = new ArrayList<>();

    private void setupScenario() {
        // Specify the global time step of the simulation.
        Simulator.instance.setTimeStep(0.25);

        // Specify the default parameters for agents that are subsequently
        // added.
        Simulator.instance.setAgentDefaults(15.0, 10, 10.0, 10.0, 1.5, 2.0, Vector2D.ZERO);

        // Add agents, specifying their start position, and store their goals on
        // the opposite side of the environment.
        double angle = 0.008 * FastMath.PI;

        for (int i = 0; i < 250; i++) {
            Simulator.instance.addAgent(new Vector2D(FastMath.cos(i * angle), FastMath.sin(i * angle)).scalarMultiply(200.0));
            goals.add(Simulator.instance.getAgentPosition(i).negate());
        }
    }

    @SuppressWarnings("SystemOut")
    private static void updateVisualization() {
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
        }
    }

    private boolean reachedGoal() {
        // Check if all agents have reached their goals.
        for (int agentNo = 0; agentNo < Simulator.instance.getNumAgents(); agentNo++) {
            if (Simulator.instance.getAgentPosition(agentNo).distanceSq(goals.get(agentNo)) > Simulator.instance.getAgentRadius(agentNo) * Simulator.instance.getAgentRadius(agentNo)) {
                return false;
            }
        }

        return true;
    }

    public static void main(String[] args) {
        Circle circle = new Circle();

        // Set up the scenario.
        circle.setupScenario();

        // Perform (and manipulate) the simulation.
        do {
            Circle.updateVisualization();
            circle.setPreferredVelocities();
            Simulator.instance.doStep();
        }
        while (!circle.reachedGoal());
    }
}
