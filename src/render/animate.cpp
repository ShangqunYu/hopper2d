#include "animate.h"

float animate()
{
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "SFML works!");
    sf::CircleShape shape(100.f);
    shape.setFillColor(sf::Color::Green);
    sf::RectangleShape line(sf::Vector2f(150.f, 5.f));
    //line.rotate(45.f);
    line.setPosition(500.f, 500.f);

    float temp = 5.0f;
    sf::Time t1 = sf::seconds(0.1f);
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        line.rotate(temp);
        window.clear();
        window.draw(shape);
        window.draw(line);
        window.display();
        sf::sleep(t1);
    }
    return 10;
}
