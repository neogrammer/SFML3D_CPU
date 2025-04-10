#pragma once
#include <SFML/Graphics.hpp>
#include <stdio.h>
#include <math.h>
#include <corecrt_math_defines.h>
#include <array>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>

constexpr uint32_t WW = 1600Ui32;
constexpr uint32_t WH = 900Ui32;


struct v3d
{
    float x, y, z;
};

struct Line
{
    sf::Vector2f start;
    sf::Vector2f end;
    sf::Vector2f direction{ 0.f, 0.f };
    float magnitude{ 0.f };


    Line(float sx_, float sy_, float ex_, float ey_) 
        : start{ sx_,sy_ }, end{ ex_,ey_ }
        , direction{  0.f,0.f }
        , magnitude{ 0.f }
    {
        findMagnitude();
        findDirection();
    }

    Line(sf::Vector2f start_, sf::Vector2f end_)
        : start{ start_ }, end{ end_ }
        , direction{ 0.f,0.f }
        , magnitude{ 0.f }
    {
        findMagnitude();
        findDirection();
    }

    sf::Vector2f getDirection()
    {
        return direction;
    }
    sf::Vector2f getStartPoint()
    {
        return start;
    }
    sf::Vector2f getEndPoint()
    {
        return end;
    }
    float getMagnitude() {
        return magnitude;
    }

private:
    void findDirection()
    {
        direction.x = (end.x - start.x) / magnitude;
        direction.y = (end.y - start.y) / magnitude;
    }
    void findMagnitude()
    {
        magnitude = sqrtf((end.x - start.x) * (end.x - start.x) + (end.y - start.y) * (end.y - start.y));
    }

};


struct Tri2D
{
    std::array<sf::Vector2f, 3> vertices;

    Tri2D(sf::Vector2f v1_, sf::Vector2f v2_, sf::Vector2f v3_)
    {
        vertices[0] = v1_;
        vertices[1] = v2_;
        vertices[2] = v3_;
    }

    Tri2D(std::array<sf::Vector2f, 3> vertices_)
        : vertices{vertices_}
    {
    }

};

struct Line3D
{
    float x;
    float y;
    float z;
};


enum class ShadeStyle
{
    PIXEL_QUARTER,
    PIXEL_HALF,
    PIXEL_THREEQUARTERS,
    PIXEL_SOLID
};

struct COLOR
{
    sf::Color bgCol;
    sf::Color fgCol;
    ShadeStyle sym;
};


struct Tri3D {
    v3d p[3]{};

    ShadeStyle sym{};
    sf::Color col{};
};


struct Mesh
{
    std::vector<Tri3D> tris;
    bool LoadFromObjectFile(std::string filename);
};

struct Mat4x4
{
    float m[4][4] = { {0.f, }, };
};

class SFML3D
{

   
    
    sf::Clock timer{};
    float fElapsedTime{ 0.f };
    float fTheta{ 0.f };
    Mat4x4 matProj{};

    v3d vCam;

    Tri2D triangle;
    Mesh cubeMesh;
    // create an empty shape
    sf::RenderWindow* pWnd{ nullptr };
public:
    static bool wireframe;
    bool drawBoth{ false };

    SFML3D(sf::RenderWindow& wnd_);
    ~SFML3D();
    bool onUserCreate(sf::RenderWindow& wnd_);
    bool onUserUpdate(float elapsedTime);
};
