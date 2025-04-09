#include "SFML3D.h"



void drawLine(Line& line_, sf::RenderWindow& wnd_)
{
    sf::RectangleShape line({ line_.getMagnitude(), 1.f });

    sf::Angle angle = sf::degrees((float)((std::atan2f((line_.direction.y), (line_.direction.x)) * (180.0f / M_PI))));

    line.rotate(angle);
    line.setPosition(line_.getStartPoint());
    line.setFillColor(sf::Color::Red);
    wnd_.draw(line);

}

void drawTriangle(Tri2D& tri_, sf::RenderWindow& wnd_, float posx_, float posy_)
{
    Line line1{ tri_.vertices[0].x + posx_, tri_.vertices[0].y + posy_,  tri_.vertices[1].x + posx_, tri_.vertices[1].y + posy_};
    Line line2{ tri_.vertices[1].x + posx_, tri_.vertices[1].y + posy_,  tri_.vertices[2].x + posx_, tri_.vertices[2].y + posy_ };
    Line line3{ tri_.vertices[0].x + posx_, tri_.vertices[0].y + posy_,  tri_.vertices[2].x + posx_, tri_.vertices[2].y + posy_ };

    drawLine(line1, wnd_);
    drawLine(line2, wnd_);
    drawLine(line3, wnd_);
}

void MulMatVec(v3d& i, v3d& o, Mat4x4& m)
{
    o.x = i.x * m.m[0][0] + i.y * m.m[1][0] + i.z * m.m[2][0] + m.m[3][0];
    o.y = i.x * m.m[0][1] + i.y * m.m[1][1] + i.z * m.m[2][1] + m.m[3][1];
    o.z = i.x * m.m[0][2] + i.y * m.m[1][2] + i.z * m.m[2][2] + m.m[3][2];
    
    float w = i.x * m.m[0][3] + i.y * m.m[1][3] + i.z * m.m[2][3] + m.m[3][3];

    if (w != 0.f)
    {
        o.x /= w;
        o.y /= w;
        o.z /= w;
    }
}

SFML3D::SFML3D(sf::RenderWindow& wnd_)
    : pWnd{&wnd_}
    , triangle{ {300.f,300.f},{400.f,500.f}, {500.f, 300.f} }
{
    onUserCreate(*pWnd);
}

SFML3D::~SFML3D()
{
}

bool SFML3D::onUserCreate(sf::RenderWindow& wnd_)
{
    pWnd = &wnd_;

    float fNear = 0.1f;
    float fFar = 1000.f;
    float fFov = 70.f;
    float fAspectRatio = (float)1 / (float)1;
    float fFovRad = 1.f / (float)(std::tanf(fFov * 0.5f / 180.f * (float)M_PI));

    matProj.m[0][0] = fAspectRatio * fFovRad;
    matProj.m[1][1] = fFovRad;
    matProj.m[2][2] = fFar / (fFar - fNear);
    matProj.m[2][3] = 1.f;
    matProj.m[3][3] = 0.f;

    cubeMesh.tris = std::vector<Tri3D>{
        Tri3D{v3d{0.f,0.f,0.f},v3d{0.f,1.f,0.f},v3d{1.f,1.f,0.f}},Tri3D{v3d{0.f,0.f,0.f},v3d{1.f,1.f,0.f},v3d{1.f,0.f,0.f}},  //Front
        Tri3D{v3d{1.f,0.f,1.f},v3d{1.f,1.f,1.f},v3d{0.f,1.f,1.f}},Tri3D{v3d{1.f,0.f,1.f},v3d{0.f,1.f,1.f},v3d{0.f,0.f,1.f}},  // Back
        Tri3D{v3d{1.f,0.f,0.f},v3d{1.f,1.f,0.f},v3d{1.f,1.f,1.f}},Tri3D{v3d{1.f,0.f,0.f},v3d{1.f,1.f,1.f},v3d{1.f,0.f,1.f}},  // East
        Tri3D{v3d{0.f,0.f,1.f},v3d{0.f,1.f,1.f},v3d{0.f,1.f,0.f}},Tri3D{v3d{0.f,0.f,1.f},v3d{0.f,1.f,0.f},v3d{0.f,0.f,0.f}},  // West
        Tri3D{v3d{0.f,1.f,0.f},v3d{0.f,1.f,1.f},v3d{1.f,1.f,1.f}},Tri3D{v3d{0.f,1.f,0.f},v3d{1.f,1.f,1.f},v3d{1.f,1.f,0.f}}, // Top
        Tri3D{v3d{0.f,0.f,1.f},v3d{0.f,0.f,0.f},v3d{1.f,0.f,0.f}},Tri3D{v3d{0.f,0.f,1.f},v3d{1.f,0.f,0.f},v3d{1.f,0.f,1.f}} // Bottom
    };

    return true;
}

bool SFML3D::onUserUpdate(float elapsedTime)
{
    pWnd->clear();

    float sizex = 300.f;
    float sizey = 300.f;
    float posx = 800.f;
    float posy = 450.f;
    fElapsedTime = timer.restart().asSeconds();
    Mat4x4 matRotZ, matRotX;
    fTheta += 1.f * fElapsedTime;
    if (fTheta >= 360.f)
    {
        fTheta = 0.f;
    }
    // RotationZ
    matRotZ.m[0][0] = std::cosf(fTheta);
    matRotZ.m[0][1] = std::sinf(fTheta);
    matRotZ.m[1][0] = -std::sinf(fTheta);
    matRotZ.m[1][1] = std::cosf(fTheta);
    matRotZ.m[2][2] = 1.f;
    matRotZ.m[3][3] = 1.f;

    // RotationX
    matRotX.m[0][0] = 1.f;
    matRotX.m[1][1] = std::cosf(fTheta*0.5f);
    matRotX.m[1][2] = std::sinf(fTheta*0.5f);
    matRotX.m[2][1] = -std::sinf(fTheta*0.5f);
    matRotX.m[2][2] = std::cosf(fTheta * 0.5f);
    matRotX.m[3][3] = 1.f;
    for (auto& tri : cubeMesh.tris)
    {
        Tri3D triProjected, triTranslated, triRotateZ, triRotateZX;
        MulMatVec(tri.p[0], triRotateZ.p[0], matRotZ);
        MulMatVec(tri.p[1], triRotateZ.p[1], matRotZ);
        MulMatVec(tri.p[2], triRotateZ.p[2], matRotZ);
        MulMatVec(tri.p[0], triRotateZX.p[0], matRotX);
        MulMatVec(tri.p[1], triRotateZX.p[1], matRotX);
        MulMatVec(tri.p[2], triRotateZX.p[2], matRotX);

        triTranslated = triRotateZX;
      // triTranslated.p[0].x = (triRotateZX.p[0].x*sizex);
      // triTranslated.p[0].y = (triRotateZX.p[0].y*-sizey);
        triTranslated.p[0].z = triRotateZX.p[0].z + 3.f;
      // triTranslated.p[1].x = (triRotateZX.p[1].x*sizex);
       //triTranslated.p[1].y = (triRotateZX.p[1].y*-sizey);
        triTranslated.p[1].z = triRotateZX.p[1].z + 3.f;
      // triTranslated.p[2].x = (triRotateZX.p[2].x*sizex);
       //triTranslated.p[2].y = (triRotateZX.p[2].y*-sizey);
        triTranslated.p[2].z = triRotateZX.p[2].z + 3.f;

        MulMatVec(triTranslated.p[0], triProjected.p[0], matProj);
        MulMatVec(triTranslated.p[1], triProjected.p[1], matProj);
        MulMatVec(triTranslated.p[2], triProjected.p[2], matProj);

        triProjected.p[0].x = (triProjected.p[0].x * sizex);
        triProjected.p[0].y = (triProjected.p[0].y * -sizey);
       // triProjected.p[0].z = triRotateZX.p[0].z + 3.f;
        triProjected.p[1].x = (triProjected.p[1].x * sizex);
        triProjected.p[1].y = (triProjected.p[1].y * -sizey);
       // triProjected.p[1].z = triRotateZX.p[1].z + 3.f;
        triProjected.p[2].x = (triProjected.p[2].x * sizex);
        triProjected.p[2].y = (triProjected.p[2].y * -sizey);
      //  triProjected.p[2].z = triRotateZX.p[2].z + 3.f;

        Tri2D tri2d{ {triProjected.p[0].x, triProjected.p[0].y},{triProjected.p[1].x,triProjected.p[1].y}, {triProjected.p[2].x, triProjected.p[2].y}};
       // Tri2D tri2d{ {posx + (0.f * sizex), posy + (0.f * -sizey)},{posx + (0.5f * sizex), posy + (1.f * sizey)}, {posx + (1.f * sizex), posy + (0.f * sizey)} };

        drawTriangle(tri2d, *pWnd, posx, posy);
    }
   
    pWnd->display();


    return true;
}
