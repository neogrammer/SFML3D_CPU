#include <optional>
#include <app/SFML3D.h>

int main()
{

    sf::ContextSettings settings;
    settings.antiAliasingLevel = 8;


    sf::RenderWindow window(sf::VideoMode({ WW, WH }), "SFML works!", sf::State::Windowed, settings);

    sf::CircleShape shape(100.f);
    shape.setFillColor(sf::Color::Green);

    SFML3D app{window};

    float elapsedTime{0.f};
    sf::Clock timer{};

    while (window.isOpen())
    {
        elapsedTime = timer.restart().asSeconds();
        if (elapsedTime > 0.016f)
            elapsedTime = 0.016f;

        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();

            else if (const auto* keyReleased = event->getIf<sf::Event::KeyReleased>())
                {
                if (keyReleased->scancode == sf::Keyboard::Scancode::Y)
                {
                    SFML3D::wireframe = !SFML3D::wireframe;
                }
                else  if (keyReleased->scancode == sf::Keyboard::Scancode::H)
                {
                    app.drawBoth = !app.drawBoth;
                }
            }
        }
        app.onUserUpdate(elapsedTime);
    }
    return 69;
}
