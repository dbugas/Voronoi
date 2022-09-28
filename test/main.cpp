/* MyGAL
 * Copyright (C) 2019 Pierre Vigier
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 // STL
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
// SFML
#include <SFML/Graphics.hpp>
// MyGAL
#include "FortuneAlgorithm.h"
#include "Vector2.h"

using namespace mygal;

using Float = double;

constexpr Float WindowWidth = 600.0f*2.0f;
constexpr Float WindowHeight = 600.0f*2.0f;
constexpr Float PointRadius = 0.005f;
constexpr Float Offset = 1.0f;

// Points generation

template<typename T>
std::vector<Vector2<T>> generatePoints(int nbPoints)
{
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::cout << "seed: " << seed << '\n';
    auto generator = std::default_random_engine(seed);
    auto distribution = std::uniform_real_distribution<T>(0.0, 1.0);

    auto points = std::vector<Vector2<T>>(nbPoints);
    for (auto i = 0; i < nbPoints; ++i)
        points[i] = Vector2<T>(distribution(generator), distribution(generator));

    return points;
}

// Rendering

template<typename T>
void drawPoint(sf::RenderWindow& window, Vector2<T> point, sf::Color color)
{
    auto shape = sf::CircleShape(PointRadius);
    shape.setPosition(sf::Vector2f(point.x - PointRadius, 1.0 - point.y - PointRadius));
    shape.setFillColor(color);
    window.draw(shape);
}

template<typename T>
void drawEdge(sf::RenderWindow& window, Vector2<T> origin, Vector2<T> destination, sf::Color color)
{
    auto line = std::array<sf::Vertex, 2>
    {
        sf::Vertex(sf::Vector2f(origin.x, 1.0 - origin.y), color),
            sf::Vertex(sf::Vector2f(destination.x, 1.0 - destination.y), color)
    };
    window.draw(line.data(), 2, sf::Lines);
}

template<typename T>
void drawPoints(sf::RenderWindow& window, const Diagram<T>& diagram)
{
    for (const auto& site : diagram.getSites())
        drawPoint(window, site.point, sf::Color(100, 250, 50));
}

template<typename T>
void drawDiagram(sf::RenderWindow& window, const Diagram<T>& diagram)
{
    for (const auto& site : diagram.getSites())
    {
        auto center = site.point;
        auto face = site.face;
        auto halfEdge = face->outerComponent;
        if (halfEdge == nullptr)
            continue;
        while (halfEdge->prev != nullptr)
        {
            halfEdge = halfEdge->prev;
            if (halfEdge == face->outerComponent)
                break;
        }
        auto start = halfEdge;
        while (halfEdge != nullptr)
        {
            if (halfEdge->origin != nullptr && halfEdge->destination != nullptr)
            {
                auto origin = (halfEdge->origin->point - center) * Offset + center;
                auto destination = (halfEdge->destination->point - center) * Offset + center;
                drawEdge(window, origin, destination, sf::Color::Blue);
            }
            halfEdge = halfEdge->next;
            if (halfEdge == start)
                break;
        }
    }
}

template<typename T>
void drawTriangulation(sf::RenderWindow& window, const Diagram<T>& diagram, const Triangulation& triangulation)
{
    for (auto i = std::size_t(0); i < diagram.getNbSites(); ++i)
    {
        auto origin = diagram.getSite(i)->point;
        for (const auto& j : triangulation.getNeighbors(i))
        {
            auto destination = diagram.getSite(j)->point;
            drawEdge(window, origin, destination, sf::Color::Green);
        }
    }
}

// Generating the diagram

template<typename T>
Diagram<T> generateDiagram(const std::vector<Vector2<T>>& points)
{
    // Construct diagram
    auto algorithm = FortuneAlgorithm<T>(points);
    auto start = std::chrono::steady_clock::now();
    algorithm.construct();
    auto duration = std::chrono::steady_clock::now() - start;
    std::cout << "construction: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << '\n';

    // Bound the diagram
    start = std::chrono::steady_clock::now();
    algorithm.bound(Box<T>{-0.05, -0.05, 1.05, 1.05}); // Take the bounding box slightly bigger than the intersection box
    duration = std::chrono::steady_clock::now() - start;
    std::cout << "bounding: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << '\n';
    auto diagram = algorithm.getDiagram();

    // Intersect the diagram with a box
    start = std::chrono::steady_clock::now();
    diagram.intersect(Box<T>{0.0, 0.0, 1.0, 1.0});
    duration = std::chrono::steady_clock::now() - start;
    std::cout << "intersection: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << '\n';

    return diagram;
}

struct graph
{
    std::vector<int> s;
    std::vector<int> t;
    int edges;
    int N;
    std::vector<std::vector<mygal::Vector2<double>>> vertices;
    std::vector<double> area;
    std::vector<double> angle;
    std::vector<double> GBarea;
};

template<typename T>
graph MakeGraph(const Diagram<T>& dia,int pnts)
{
    graph G;
    G.area = dia.computeArea(); // construct graph
    G.vertices = dia.ReturnV(); // get vertices of each polygon
    //G.angle = dia.computeAngle(); // get inclination of edge in radians
    auto triangulation = dia.computeTriangulation();  // get area of each polygon
    
    for (int i = 0; i < pnts; i++) {
        auto& Neighbor = triangulation.getNeighbors(i);
        for (int j = 0; j < Neighbor.size(); j++) {
            G.s.push_back(i);
            G.t.push_back(Neighbor.at(j));
        }
    }
    double tol = 1e-20;
    std::vector<mygal::Vector2<double>> triPnt;
    for (int i = 0; i < G.s.size(); i++) {
        std::vector<mygal::Vector2<double>> Vs = G.vertices.at(G.s.at(i));
        std::vector<mygal::Vector2<double>> Vt = G.vertices.at(G.t.at(i));

        triPnt.clear();
        for (int j = 0; j < Vs.size(); j++) {
            for (int k = 0; k < Vt.size(); k++) {

                double testx = (Vs.at(j) - Vt.at(k)).getNorm();
                double testy = (Vs.at(j) - Vt.at(k)).getNorm() <= tol;
                if ((Vs.at(j)-Vt.at(k)).getNorm() <= tol) {

                    triPnt.push_back(Vs.at(j));
                    if (triPnt.size() >= 2) {
                    
                        G.GBarea.push_back((triPnt.at(0) - triPnt.at(1)).getNorm());
                        G.angle.push_back(triPnt.at(0).getAngle(triPnt.at(1)));
                        break;
                    }
                }
            }
        }
    }
    G.edges = G.s.size();
    G.N = G.edges + pnts;
    return G;
}

int main()
{
    auto nbPoints = 100;
    auto diagram = generateDiagram(generatePoints<Float>(nbPoints));
    auto triangulation = diagram.computeTriangulation();

    graph G = MakeGraph(diagram, nbPoints);
 
    // Display the diagram
    auto settings = sf::ContextSettings();
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WindowWidth, WindowHeight), "MyGAL", sf::Style::Default, settings); // Can use auto only in C++17
    window.setView(sf::View(sf::FloatRect(-0.1f, -0.1f, 1.2f, 1.2f)));
    window.setFramerateLimit(60);
    auto showTriangulation = false;

    while (window.isOpen())
    {
        auto event = sf::Event();
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            else if (event.type == sf::Event::KeyReleased)
            {
                if (event.key.code == sf::Keyboard::Key::N)
                {
                    diagram = generateDiagram(generatePoints<Float>(nbPoints));
                    triangulation = diagram.computeTriangulation();
                }
                else if (event.key.code == sf::Keyboard::Key::R)
                {
                    diagram = generateDiagram(diagram.computeLloydRelaxation());
                    triangulation = diagram.computeTriangulation();
                }
                else if (event.key.code == sf::Keyboard::Key::T)
                    showTriangulation = !showTriangulation;
            }
        }

        window.clear(sf::Color::White);

        if (!showTriangulation)
            drawDiagram(window, diagram);
        //drawPoints(window, diagram);
        if (showTriangulation)
            drawTriangulation(window, diagram, triangulation);

        window.display();
    }

    return 0;
}
