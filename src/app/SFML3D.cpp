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
    Line line1{ tri_.vertices[0].x , tri_.vertices[0].y ,  tri_.vertices[1].x , tri_.vertices[1].y };
    Line line2{ tri_.vertices[1].x , tri_.vertices[1].y ,  tri_.vertices[2].x , tri_.vertices[2].y  };
    Line line3{ tri_.vertices[2].x , tri_.vertices[2].y ,  tri_.vertices[0].x , tri_.vertices[0].y  };

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


    cubeMesh.tris = {
        {0.f,0.f,0.f,  0.f,1.f,0.f,  1.f,1.f,0.f },
        {0.f,0.f,0.f,  1.f,1.f,0.f,  1.f,0.f,0.f},  //Front

        {1.f,0.f,1.f,  1.f,1.f,1.f,  0.f,1.f,1.f },
        {1.f,0.f,1.f,  0.f,1.f,1.f,  0.f,0.f,1.f},  // Back

        {1.f,0.f,0.f,  1.f,1.f,0.f,  1.f,1.f,1.f },
        {1.f,0.f,0.f,  1.f,1.f,1.f,  1.f,0.f,1.f},  // East

        {0.f,0.f,1.f,  0.f,1.f,1.f,  0.f,1.f,0.f },
        {0.f,0.f,1.f,  0.f,1.f,0.f,  0.f,0.f,0.f},  // West

        {0.f,1.f,0.f,  0.f,1.f,1.f,  1.f,1.f,1.f },
        {0.f,1.f,0.f,  1.f,1.f,1.f,  1.f,1.f,0.f}, // Top
        
        {0.f,0.f,1.f,  0.f,0.f,0.f,  1.f,0.f,0.f },
        {0.f,0.f,1.f,  1.f,0.f,0.f,  1.f,0.f,1.f} // Bottom
    };

    float fNear = 0.1f;
    float fFar = 1000.f;
    float fFov = 50.f;
    float fAspectRatio = (float)WH / (float)WW;
    float fFovRad = 1.f / (float)(std::tanf(fFov * 0.5f / 180.f * (float)M_PI));

    matProj.m[0][0] = fAspectRatio * fFovRad;
    matProj.m[1][1] = fFovRad;
    matProj.m[2][2] = fFar / (fFar - fNear);
    matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
    matProj.m[2][3] = 1.f;
    matProj.m[3][3] = 0.f;

    return true;
}

bool SFML3D::onUserUpdate(float elapsedTime)
{
    pWnd->clear(sf::Color(sf::Color::Blue));

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
        Tri3D triProjected, triTranslated, triRotatedZ, triRotatedZX;
        
        MulMatVec(tri.p[0], triRotatedZ.p[0], matRotZ);
        MulMatVec(tri.p[1], triRotatedZ.p[1], matRotZ);
        MulMatVec(tri.p[2], triRotatedZ.p[2], matRotZ);

        MulMatVec(triRotatedZ.p[0], triRotatedZX.p[0], matRotX);
        MulMatVec(triRotatedZ.p[1], triRotatedZX.p[1], matRotX);
        MulMatVec(triRotatedZ.p[2], triRotatedZX.p[2], matRotX);

        triTranslated = triRotatedZX;
        triTranslated.p[0].z = triRotatedZX.p[0].z + 3.f;
        triTranslated.p[1].z = triRotatedZX.p[1].z + 3.f;
        triTranslated.p[2].z = triRotatedZX.p[2].z + 3.f;


      //  
      //  
      //  

      // 
      // 
      // 

      //  triTranslated = triRotateZX;
      //// triTranslated.p[0].x = (triRotateZX.p[0].x*sizex);
      //// triTranslated.p[0].y = (triRotateZX.p[0].y*-sizey);
      //  triTranslated.p[0].z = triRotateZX.p[0].z + 3.f;
      //// triTranslated.p[1].x = (triRotateZX.p[1].x*sizex);
      // //triTranslated.p[1].y = (triRotateZX.p[1].y*-sizey);
      //  triTranslated.p[1].z = triRotateZX.p[1].z + 3.f;
      //// triTranslated.p[2].x = (triRotateZX.p[2].x*sizex);
      // //triTranslated.p[2].y = (triRotateZX.p[2].y*-sizey);
      //  triTranslated.p[2].z = triRotateZX.p[2].z + 3.f;

        MulMatVec(triTranslated.p[0], triProjected.p[0], matProj);
        MulMatVec(triTranslated.p[1], triProjected.p[1], matProj);
        MulMatVec(triTranslated.p[2], triProjected.p[2], matProj);

        triProjected.p[0].x += 1.f; triProjected.p[0].y += 1.f;
        triProjected.p[1].x += 1.f; triProjected.p[1].y += 1.f;
        triProjected.p[2].x += 1.f; triProjected.p[2].y += 1.f;

        triProjected.p[0].x *= 0.5f * (float)WW;
        triProjected.p[0].y *= 0.5f * (float)WH;
        triProjected.p[1].x *= 0.5f * (float)WW;
        triProjected.p[1].y *= 0.5f * (float)WH;
        triProjected.p[2].x *= 0.5f * (float)WW;
        triProjected.p[2].y *= 0.5f * (float)WH;



       // triProjected.p[0].x = (triProjected.p[0].x * sizex);
       // triProjected.p[0].y = (triProjected.p[0].y * -sizey);
       //// triProjected.p[0].z = triRotateZX.p[0].z + 3.f;
       // triProjected.p[1].x = (triProjected.p[1].x * sizex);
       // triProjected.p[1].y = (triProjected.p[1].y * -sizey);
       //// triProjected.p[1].z = triRotateZX.p[1].z + 3.f;
       // triProjected.p[2].x = (triProjected.p[2].x * sizex);
       // triProjected.p[2].y = (triProjected.p[2].y * -sizey);
      //  triProjected.p[2].z = triRotateZX.p[2].z + 3.f;

        Tri2D tri2d{ {triProjected.p[0].x, triProjected.p[0].y},{triProjected.p[1].x,triProjected.p[1].y}, {triProjected.p[2].x, triProjected.p[2].y}};
       // Tri2D tri2d{ {posx + (0.f * sizex), posy + (0.f * -sizey)},{posx + (0.5f * sizex), posy + (1.f * sizey)}, {posx + (1.f * sizex), posy + (0.f * sizey)} };

        drawTriangle(tri2d, *pWnd, 0.f,0.f);
    }
   
    pWnd->display();


    return true;
}
