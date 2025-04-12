#include "SFML3D.h"
#include <strstream>
#include <algorithm>

#define GLM_FORCE_RADIANS
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>


bool SFML3D::wireframe = true;


float dot(v3d& a, v3d& b)
{
    float l1 = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
    v3d vec1{ a.x / l1, a.y / l1, a.z / l1 };
    float l2 = sqrtf(b.x * b.x + b.y * b.y + b.z * b.z);
    v3d vec2{ b.x / l2, b.y / l2, b.z / l2 };

    return (vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z);

}

v3d crossProd(Line3D& a, Line3D& b)
{
    v3d n{ 0.f,0.f,0.f };
    n.x = a.y * b.z - a.z * b.y;
    n.y = a.z * b.x - a.x * b.z;
    n.z = a.x * b.y - a.y * b.x;

    return n;

}

void drawLine(Line& line_, sf::RenderWindow& wnd_, sf::Color col_ = sf::Color::Red)
{
    sf::RectangleShape line({ line_.getMagnitude(), 1.f });

    sf::Angle angle = sf::degrees((float)((std::atan2f((line_.direction.y), (line_.direction.x)) * (180.0f / M_PI))));

    line.rotate(angle);
    line.setPosition(line_.getStartPoint());
    line.setFillColor(col_);
    wnd_.draw(line);

}

void drawTriangle(Tri2D& tri_, sf::RenderWindow& wnd_, sf::Color col_, bool drawBoth = true)
{
  
    if (!SFML3D::wireframe)
    {
        if (!drawBoth)
        {

            sf::ConvexShape triangle{ 3 };
            triangle.setPoint(0, tri_.vertices[0]);
            triangle.setPoint(1, tri_.vertices[1]);
            triangle.setPoint(2, tri_.vertices[2]);
            triangle.setFillColor(col_);


            wnd_.draw(triangle);
        }
        else
        {

            sf::ConvexShape triangle{ 3 };
            triangle.setPoint(0, tri_.vertices[0]);
            triangle.setPoint(1, tri_.vertices[1]);
            triangle.setPoint(2, tri_.vertices[2]);
            triangle.setFillColor(col_);


            wnd_.draw(triangle);
            Line line1{ tri_.vertices[0].x , tri_.vertices[0].y ,  tri_.vertices[1].x , tri_.vertices[1].y };
            Line line2{ tri_.vertices[1].x , tri_.vertices[1].y ,  tri_.vertices[2].x , tri_.vertices[2].y };
            Line line3{ tri_.vertices[2].x , tri_.vertices[2].y ,  tri_.vertices[0].x , tri_.vertices[0].y };

            drawLine(line1, wnd_, sf::Color::Black);
            drawLine(line2, wnd_, sf::Color::Black);
            drawLine(line3, wnd_, sf::Color::Black);


        }
    }
    else
    {
        if (!drawBoth)
        {
            Line line1{ tri_.vertices[0].x , tri_.vertices[0].y ,  tri_.vertices[1].x , tri_.vertices[1].y };
            Line line2{ tri_.vertices[1].x , tri_.vertices[1].y ,  tri_.vertices[2].x , tri_.vertices[2].y };
            Line line3{ tri_.vertices[2].x , tri_.vertices[2].y ,  tri_.vertices[0].x , tri_.vertices[0].y };

            drawLine(line1, wnd_, col_);
            drawLine(line2, wnd_, col_);
            drawLine(line3, wnd_, col_);
        }
        else
        {
            sf::ConvexShape triangle{ 3 };
            triangle.setPoint(0, tri_.vertices[0]);
            triangle.setPoint(1, tri_.vertices[1]);
            triangle.setPoint(2, tri_.vertices[2]);
            triangle.setFillColor(col_);


            wnd_.draw(triangle);
            Line line1{ tri_.vertices[0].x , tri_.vertices[0].y ,  tri_.vertices[1].x , tri_.vertices[1].y };
            Line line2{ tri_.vertices[1].x , tri_.vertices[1].y ,  tri_.vertices[2].x , tri_.vertices[2].y };
            Line line3{ tri_.vertices[2].x , tri_.vertices[2].y ,  tri_.vertices[0].x , tri_.vertices[0].y };

            drawLine(line1, wnd_, sf::Color::Black);
            drawLine(line2, wnd_, sf::Color::Black);
            drawLine(line3, wnd_, sf::Color::Black);

        }
    }
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



COLOR getColor(float lum)
{
    sf::Color bgCol, fgCol;
    ShadeStyle sym;
    int pixel_bw = (int)(13.0f * lum);
    switch (pixel_bw)        
    {
    case 0:             bgCol = { sf::Color::Black };                     fgCol = { sf::Color::Black };              sym = ShadeStyle::PIXEL_SOLID;   break;
    case 1:             bgCol = { sf::Color::Black };                     fgCol = { sf::Color::Blue };               sym = ShadeStyle::PIXEL_QUARTER;   break;
    case 2:             bgCol = { sf::Color::Black };                     fgCol = { sf::Color::Blue };               sym = ShadeStyle::PIXEL_HALF;   break;
    case 3:             bgCol = { sf::Color::Black };                     fgCol = { sf::Color::Blue };               sym = ShadeStyle::PIXEL_THREEQUARTERS;   break;
    case 4:             bgCol = { sf::Color::Black };                     fgCol = { sf::Color::Blue };               sym = ShadeStyle::PIXEL_SOLID;   break;
    case 5:             bgCol = { sf::Color::Blue };                      fgCol = { sf::Color(46,146,246,255) };     sym = ShadeStyle::PIXEL_QUARTER;   break;
        case 6:             bgCol = { sf::Color::Black };                     fgCol = { sf::Color(46,146,246,255) };     sym = ShadeStyle::PIXEL_HALF;   break;
        case 7:             bgCol = { sf::Color::Black };                     fgCol = { sf::Color(46,146,246,255) };     sym = ShadeStyle::PIXEL_THREEQUARTERS;   break;
        case 8:             bgCol = { sf::Color::Black };                     fgCol = { sf::Color(46,146,246,255) };     sym = ShadeStyle::PIXEL_SOLID;   break;
        case 9:             bgCol = { sf::Color(46,146,246,255) };            fgCol = { sf::Color::White };              sym = ShadeStyle::PIXEL_QUARTER;   break;
        case 10:            bgCol = { sf::Color(46,146,246,255) };            fgCol = { sf::Color::White };              sym = ShadeStyle::PIXEL_HALF;   break;
        case 11:            bgCol = { sf::Color(46,146,246,255) };            fgCol = { sf::Color::White };              sym = ShadeStyle::PIXEL_THREEQUARTERS;   break;
        case 12:            bgCol = { sf::Color(46,146,246,255) };            fgCol = { sf::Color::White };              sym = ShadeStyle::PIXEL_SOLID;   break;
        default:            bgCol = sf::Color::Black;                         fgCol = sf::Color::Black;                  sym = ShadeStyle::PIXEL_SOLID;  break;
    }
    std::uint8_t r, g, b;
    r = 255;
    g = 0;
    b = 0;
    r = std::uint8_t(lum * (float)r);
    g = std::uint8_t(lum * (float)g);
    b = std::uint8_t(lum * (float)b);

    bgCol = sf::Color(r, g, b);
    fgCol = sf::Color(r, g, b);
    sym = ShadeStyle::PIXEL_SOLID;

    return { bgCol,fgCol, sym };
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


    //cubeMesh.LoadFromObjectFile("models/TorusZforward.cid");
    cubeMesh.tris = {
        { 0.f, 0.f, 0.f,   0.f, 1.f, 0.f,   1.f, 1.f, 0.f}, 
        {0.f, 0.f, 0.f,  1.f, 1.f, 0.f,   1.f, 0.f, 0.f}, // FRONT

        { 1.f, 0.f, 1.f,   1.f, 1.f, 1.f,   0.f, 1.f, 1.f},
        {1.f, 0.f, 1.f,  0.f, 1.f, 1.f,   0.f, 0.f, 1.f}, // BACK

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
    float fFov = 87.f;
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
    { fTheta = 0.f; }
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

    std::vector<Tri3D> vecTrianglesToRaster;


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


        v3d normal;
        Line3D line1{ triTranslated.p[1].x - triTranslated.p[0].x, triTranslated.p[1].y - triTranslated.p[0].y, triTranslated.p[1].z - triTranslated.p[0].z };
        Line3D line2{ triTranslated.p[2].x - triTranslated.p[0].x, triTranslated.p[2].y - triTranslated.p[0].y, triTranslated.p[2].z - triTranslated.p[0].z };

        normal = crossProd(line1, line2);
        
        float l = sqrtf(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
        normal.x /= 1.f; normal.y /= 1.f; normal.z /= 1.f;
        
      /*  if (normal.z < 0)*/
        if(normal.x*(triTranslated.p[0].x - vCam.x) +
            normal.y * (triTranslated.p[0].y - vCam.y) +
            normal.z * (triTranslated.p[0].z - vCam.z) < 0.0f)
        {
            // Illumination
            v3d lightDirection = { 0.f, 0.f, -1.f };
            float l = sqrtf(lightDirection.x * lightDirection.x + lightDirection.y * lightDirection.y + lightDirection.z * lightDirection.z);
            lightDirection.x /= 1.f; lightDirection.y /= 1.f; lightDirection.z /= 1.f;

            float dp = normal.x * lightDirection.x + normal.y * lightDirection.y + normal.z * lightDirection.z;

            COLOR c = getColor(dp);
            triTranslated.col = c.bgCol;
            triTranslated.sym = ShadeStyle::PIXEL_SOLID;


            MulMatVec(triTranslated.p[0], triProjected.p[0], matProj);
            MulMatVec(triTranslated.p[1], triProjected.p[1], matProj);
            MulMatVec(triTranslated.p[2], triProjected.p[2], matProj);

            triProjected.col = triTranslated.col;
            triProjected.sym = triTranslated.sym;


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

            vecTrianglesToRaster.push_back(triProjected);


            //Tri2D tri2d{ {triProjected.p[0].x, triProjected.p[0].y},{triProjected.p[1].x,triProjected.p[1].y}, {triProjected.p[2].x, triProjected.p[2].y} };
            //// Tri2D tri2d{ {posx + (0.f * sizex), posy + (0.f * -sizey)},{posx + (0.5f * sizex), posy + (1.f * sizey)}, {posx + (1.f * sizex), posy + (0.f * sizey)} };

            //drawTriangle(tri2d, *pWnd, triProjected.col, drawBoth);
        }


    }
   
    std::sort(vecTrianglesToRaster.begin(), vecTrianglesToRaster.end(), [](Tri3D& t1, Tri3D& t2)
        {
            float z1 = (t1.p[0].z + t1.p[1].z + t1.p[2].z) / 3.f;
            float z2 = (t2.p[0].z + t2.p[1].z + t2.p[2].z) / 3.f;
            return z1 > z2;
        });

    for (auto& triProjected : vecTrianglesToRaster)
    {
        // rasterize triangle
        Tri2D tri2d{ {triProjected.p[0].x, triProjected.p[0].y},{triProjected.p[1].x,triProjected.p[1].y}, {triProjected.p[2].x, triProjected.p[2].y} };
        // Tri2D tri2d{ {posx + (0.f * sizex), posy + (0.f * -sizey)},{posx + (0.5f * sizex), posy + (1.f * sizey)}, {posx + (1.f * sizex), posy + (0.f * sizey)} };

        drawTriangle(tri2d, *pWnd, triProjected.col, drawBoth);
    }



    pWnd->display();


    return true;
}

bool Mesh::LoadFromObjectFile(std::string filename)
{
    
    std::ifstream f(filename);
    if (!f.is_open())
        return false;

    // local cache of verts
    std::vector<v3d> verts;

    while (!f.eof())
    {
        char line[128];
        f.getline(line, 128);

        std::strstream s;
        s << line;
        char junk;
        if (line[0] == 'v')
        {
            v3d v;
            s >> junk >> v.x >> v.y >> v.z;
            verts.push_back(v);
        }

        if (line[0] == 'f')
        {
            int f[3];
            s >> junk >> f[0] >> f[1] >> f[2];
            tris.push_back({ verts[f[0] - 1], verts[f[1] - 1], verts[f[2] - 1] });
        }

        
    }



    return true;
}
