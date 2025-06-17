#include <SFML/Graphics.hpp>
#include <unordered_map>
#include <cmath>
#include <string>

// Aircraft representation
struct Aircraft {
    std::string icao;
    float lat;
    float lon;
    sf::Sprite sprite;
};

// Metro Vancouver bounding box (approximate)
//MIN_LAT, MIN_LON = 49.0, -123.3  # Southwest (Richmond, Delta)
//MAX_LAT, MAX_LON = 49.5, -122.5  # Northeast (North Van, Coquitlam)

// Map bounds (adjust to match your actual map)
const float TOP_LAT    = 49.5f;
const float BOTTOM_LAT = 49.0f;
const float LEFT_LON   = -123.3;
const float RIGHT_LON  = -122.5;

sf::Vector2f latlonToPixel(float lat, float lon, int mapWidth, int mapHeight) {
    float x = (lon - LEFT_LON) / (RIGHT_LON - LEFT_LON) * mapWidth;
    float y = (TOP_LAT - lat) / (TOP_LAT - BOTTOM_LAT) * mapHeight;
    return sf::Vector2f(x, y);
}

int main() {
    sf::RenderWindow window(sf::VideoMode(1000, 800), "ADS-B Real-Time Plot");

    sf::Texture mapTex;
    if (!mapTex.loadFromFile("map.png"))
        return -1;
    sf::Sprite mapSprite(mapTex);

    sf::Texture planeTex;
    if (!planeTex.loadFromFile("red.png"))
        return -1;

    std::unordered_map<std::string, Aircraft> aircrafts;

    sf::Clock clock;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event))
            if (event.type == sf::Event::Closed)
                window.close();

        //  Simulate aircraft (replace with real ADS-B parsing)
        float t = clock.getElapsedTime().asSeconds();
        float lat = 39.8f + std::sin(t * 0.05f) * 0.1f;
        float lon = -104.9f + std::cos(t * 0.05f) * 0.1f;

       // {49.1913, -122.8490}, // Surrey
        Aircraft ac1;
        ac1.icao = "ABC123";
        ac1.lat = 49.1913;
        ac1.lon = -122.8490;
        ac1.sprite.setTexture(planeTex);
        ac1.sprite.setScale(0.05f, 0.05f); // make icon small
        
        sf::Vector2f pos1 = latlonToPixel(ac1.lat, ac1.lon, mapTex.getSize().x, mapTex.getSize().y);
        ac1.sprite.setPosition(pos1);
        aircrafts[ac1.icao] = ac1;

         // {49.1666, -123.1336}, // Richmond
        Aircraft ac2;
        ac2.icao = "XYZ123";
        ac2.lat = 49.1666;
        ac2.lon = -123.1336;
        ac2.sprite.setTexture(planeTex);
        ac2.sprite.setScale(0.05f, 0.05f); // make icon small
        
        sf::Vector2f pos2 = latlonToPixel(ac2.lat, ac2.lon, mapTex.getSize().x, mapTex.getSize().y);
        ac2.sprite.setPosition(pos2);
        aircrafts[ac2.icao] = ac2;

        //  Render
        window.clear();
        window.draw(mapSprite);
        for (auto& [_, a] : aircrafts)
            window.draw(a.sprite);
        window.display();
    }

    return 0;
}
