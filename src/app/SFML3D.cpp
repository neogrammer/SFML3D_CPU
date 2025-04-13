#include "SFML3D.h"
#include <strstream>
#include <algorithm>

#define GLM_FORCE_RADIANS
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>


bool SFML3D::wireframe = true;
sf::Texture SFML3D::texGrass;
sf::Texture SFML3D::texTop;
sf::Texture SFML3D::texSide;
sf::Texture SFML3D::texBottom;



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
            if (tri_.faceType == FaceType::Side)
                triangle.setTexture(&SFML3D::texSide);
            else if (tri_.faceType == FaceType::Top)
                triangle.setTexture(&SFML3D::texTop);
            else
                triangle.setTexture(&SFML3D::texBottom);

            triangle.setTextureRect({ { 0, 0 }, { 64, 64 } });
           // triangle.setFillColor(col_);


            wnd_.draw(triangle);
        }
        else
        {

            sf::ConvexShape triangle{ 3 };
            triangle.setPoint(0, tri_.vertices[0]);
            triangle.setPoint(1, tri_.vertices[1]);
            triangle.setPoint(2, tri_.vertices[2]);
            if (tri_.faceType == FaceType::Side)
                triangle.setTexture(&SFML3D::texSide);
            else if (tri_.faceType == FaceType::Top)
                triangle.setTexture(&SFML3D::texTop);
            else
                triangle.setTexture(&SFML3D::texBottom);
            triangle.setTextureRect({ { 0, 0 }, { 64, 64 } });
          //  triangle.setFillColor(col_);


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
            if (tri_.faceType == FaceType::Side)
                triangle.setTexture(&SFML3D::texSide);
            else if (tri_.faceType == FaceType::Top)
                triangle.setTexture(&SFML3D::texTop);
            else
                triangle.setTexture(&SFML3D::texBottom);
            triangle.setTextureRect({ { 0, 0 }, { 64, 64 } });


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

v3d Matrix_MultiplyVector(Mat4x4& m, v3d& i)
{
    v3d v;
    v.x = i.x * m.m[0][0] + i.y * m.m[1][0] + i.z * m.m[2][0] + i.w * m.m[3][0];
    v.y = i.x * m.m[0][1] + i.y * m.m[1][1] + i.z * m.m[2][1] + i.w * m.m[3][1];
    v.z = i.x * m.m[0][2] + i.y * m.m[1][2] + i.z * m.m[2][2] + i.w * m.m[3][2];
    v.w = i.x * m.m[0][3] + i.y * m.m[1][3] + i.z * m.m[2][3] + i.w * m.m[3][3];
    return v;
}

Mat4x4 Matrix_MakeIdentity()
{
    Mat4x4 matrix;
    matrix.m[0][0] = 1.0f;
    matrix.m[1][1] = 1.0f;
    matrix.m[2][2] = 1.0f;
    matrix.m[3][3] = 1.0f;
    return matrix;
}

Mat4x4 Matrix_MakeRotationX(float fAngleRad)
{
    Mat4x4 matrix;
    matrix.m[0][0] = 1.0f;
    matrix.m[1][1] = cosf(fAngleRad);
    matrix.m[1][2] = sinf(fAngleRad);
    matrix.m[2][1] = -sinf(fAngleRad);
    matrix.m[2][2] = cosf(fAngleRad);
    matrix.m[3][3] = 1.0f;
    return matrix;
}

Mat4x4 Matrix_MakeRotationY(float fAngleRad)
{
    Mat4x4 matrix;
    matrix.m[0][0] = cosf(fAngleRad);
    matrix.m[0][2] = sinf(fAngleRad);
    matrix.m[2][0] = -sinf(fAngleRad);
    matrix.m[1][1] = 1.0f;
    matrix.m[2][2] = cosf(fAngleRad);
    matrix.m[3][3] = 1.0f;
    return matrix;
}

Mat4x4 Matrix_MakeRotationZ(float fAngleRad)
{
    Mat4x4 matrix;
    matrix.m[0][0] = cosf(fAngleRad);
    matrix.m[0][1] = sinf(fAngleRad);
    matrix.m[1][0] = -sinf(fAngleRad);
    matrix.m[1][1] = cosf(fAngleRad);
    matrix.m[2][2] = 1.0f;
    matrix.m[3][3] = 1.0f;
    return matrix;
}

Mat4x4 Matrix_MakeTranslation(float x, float y, float z)
{
    Mat4x4 matrix;
    matrix.m[0][0] = 1.0f;
    matrix.m[1][1] = 1.0f;
    matrix.m[2][2] = 1.0f;
    matrix.m[3][3] = 1.0f;
    matrix.m[3][0] = x;
    matrix.m[3][1] = y;
    matrix.m[3][2] = z;
    return matrix;
}

Mat4x4 Matrix_MakeProjection(float fFovDegrees, float fAspectRatio, float fNear, float fFar)
{
    float fFovRad = 1.0f / tanf(fFovDegrees * 0.5f / 180.0f * 3.14159f);
    Mat4x4 matrix;
    matrix.m[0][0] = fAspectRatio * fFovRad;
    matrix.m[1][1] = fFovRad;
    matrix.m[2][2] = fFar / (fFar - fNear);
    matrix.m[3][2] = (-fFar * fNear) / (fFar - fNear);
    matrix.m[2][3] = 1.0f;
    matrix.m[3][3] = 0.0f;
    return matrix;
}

Mat4x4 Matrix_MultiplyMatrix(Mat4x4& m1, Mat4x4& m2)
{
    Mat4x4 matrix;
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            matrix.m[r][c] = m1.m[r][0] * m2.m[0][c] + m1.m[r][1] * m2.m[1][c] + m1.m[r][2] * m2.m[2][c] + m1.m[r][3] * m2.m[3][c];
    return matrix;
}



Mat4x4 Matrix_QuickInverse(Mat4x4& m) // Only for Rotation/Translation Matrices
{
    Mat4x4 matrix;
    matrix.m[0][0] = m.m[0][0]; matrix.m[0][1] = m.m[1][0]; matrix.m[0][2] = m.m[2][0]; matrix.m[0][3] = 0.0f;
    matrix.m[1][0] = m.m[0][1]; matrix.m[1][1] = m.m[1][1]; matrix.m[1][2] = m.m[2][1]; matrix.m[1][3] = 0.0f;
    matrix.m[2][0] = m.m[0][2]; matrix.m[2][1] = m.m[1][2]; matrix.m[2][2] = m.m[2][2]; matrix.m[2][3] = 0.0f;
    matrix.m[3][0] = -(m.m[3][0] * matrix.m[0][0] + m.m[3][1] * matrix.m[1][0] + m.m[3][2] * matrix.m[2][0]);
    matrix.m[3][1] = -(m.m[3][0] * matrix.m[0][1] + m.m[3][1] * matrix.m[1][1] + m.m[3][2] * matrix.m[2][1]);
    matrix.m[3][2] = -(m.m[3][0] * matrix.m[0][2] + m.m[3][1] * matrix.m[1][2] + m.m[3][2] * matrix.m[2][2]);
    matrix.m[3][3] = 1.0f;
    return matrix;
}

v3d Vector_Add(v3d& v1, v3d& v2)
{
    return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}

v3d Vector_Sub(v3d& v1, v3d& v2)
{
    return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

v3d Vector_Mul(v3d& v1, float k)
{
    return { v1.x * k, v1.y * k, v1.z * k };
}

v3d Vector_Div(v3d& v1, float k)
{
    return { v1.x / k, v1.y / k, v1.z / k };
}

float Vector_DotProduct(v3d& v1, v3d& v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

float Vector_Length(v3d& v)
{
    return sqrtf(Vector_DotProduct(v, v));
}

v3d Vector_Normalise(v3d& v)
{
    float l = Vector_Length(v);
    return { v.x / l, v.y / l, v.z / l };
}

v3d Vector_CrossProduct(v3d& v1, v3d& v2)
{
    v3d v;
    v.x = v1.y * v2.z - v1.z * v2.y;
    v.y = v1.z * v2.x - v1.x * v2.z;
    v.z = v1.x * v2.y - v1.y * v2.x;
    return v;
}

v3d Vector_IntersectPlane(v3d& plane_p, v3d& plane_n, v3d& lineStart, v3d& lineEnd)
{
    plane_n = Vector_Normalise(plane_n);
    float plane_d = -Vector_DotProduct(plane_n, plane_p);
    float ad = Vector_DotProduct(lineStart, plane_n);
    float bd = Vector_DotProduct(lineEnd, plane_n);
    float t = (-plane_d - ad) / (bd - ad);
    v3d lineStartToEnd = Vector_Sub(lineEnd, lineStart);
    v3d lineToIntersect = Vector_Mul(lineStartToEnd, t);
    return Vector_Add(lineStart, lineToIntersect);
}

Mat4x4 Matrix_PointAt(v3d& pos, v3d& target, v3d& up)
{
    // Calculate new forward direction
    v3d newForward = Vector_Sub(target, pos);
    newForward = Vector_Normalise(newForward);

    // Calculate new Up direction
    v3d a = Vector_Mul(newForward, Vector_DotProduct(up, newForward));
    v3d newUp = Vector_Sub(up, a);
    newUp = Vector_Normalise(newUp);

    // New Right direction is easy, its just cross product
    v3d newRight = Vector_CrossProduct(newUp, newForward);

    // Construct Dimensioning and Translation Matrix	
    Mat4x4 matrix;
    matrix.m[0][0] = newRight.x;	matrix.m[0][1] = newRight.y;	matrix.m[0][2] = newRight.z;	matrix.m[0][3] = 0.0f;
    matrix.m[1][0] = newUp.x;		matrix.m[1][1] = newUp.y;		matrix.m[1][2] = newUp.z;		matrix.m[1][3] = 0.0f;
    matrix.m[2][0] = newForward.x;	matrix.m[2][1] = newForward.y;	matrix.m[2][2] = newForward.z;	matrix.m[2][3] = 0.0f;
    matrix.m[3][0] = pos.x;			matrix.m[3][1] = pos.y;			matrix.m[3][2] = pos.z;			matrix.m[3][3] = 1.0f;
    return matrix;

}

int Triangle_ClipAgainstPlane(v3d plane_p, v3d plane_n, Tri3D& in_tri, Tri3D& out_tri1, Tri3D& out_tri2)
{
    // Make sure plane normal is indeed normal
    plane_n = Vector_Normalise(plane_n);

    // Return signed shortest distance from point to plane, plane normal must be normalised
    auto dist = [&](v3d& p)
        {
            v3d n = Vector_Normalise(p);
            return (plane_n.x * p.x + plane_n.y * p.y + plane_n.z * p.z - Vector_DotProduct(plane_n, plane_p));
        };

    // Create two temporary storage arrays to classify points either side of plane
    // If distance sign is positive, point lies on "inside" of plane
    v3d* inside_points[3];  int nInsidePointCount = 0;
    v3d* outside_points[3]; int nOutsidePointCount = 0;

    // Get signed distance of each point in triangle to plane
    float d0 = dist(in_tri.p[0]);
    float d1 = dist(in_tri.p[1]);
    float d2 = dist(in_tri.p[2]);

    if (d0 >= 0) { inside_points[nInsidePointCount++] = &in_tri.p[0]; }
    else { outside_points[nOutsidePointCount++] = &in_tri.p[0]; }
    if (d1 >= 0) { inside_points[nInsidePointCount++] = &in_tri.p[1]; }
    else { outside_points[nOutsidePointCount++] = &in_tri.p[1]; }
    if (d2 >= 0) { inside_points[nInsidePointCount++] = &in_tri.p[2]; }
    else { outside_points[nOutsidePointCount++] = &in_tri.p[2]; }

    // Now classify triangle points, and break the input triangle into 
    // smaller output triangles if required. There are four possible
    // outcomes...

    if (nInsidePointCount == 0)
    {
        // All points lie on the outside of plane, so clip whole triangle
        // It ceases to exist

        return 0; // No returned triangles are valid
    }

    if (nInsidePointCount == 3)
    {
        // All points lie on the inside of plane, so do nothing
        // and allow the triangle to simply pass through
        out_tri1 = in_tri;

        return 1; // Just the one returned original triangle is valid
    }

    if (nInsidePointCount == 1 && nOutsidePointCount == 2)
    {
        // Triangle should be clipped. As two points lie outside
        // the plane, the triangle simply becomes a smaller triangle

        // Copy appearance info to new triangle
        out_tri1.col = in_tri.col;
        out_tri1.sym = in_tri.sym;

        // The inside point is valid, so keep that...
        out_tri1.p[0] = *inside_points[0];

        // but the two new points are at the locations where the 
        // original sides of the triangle (lines) intersect with the plane
        out_tri1.p[1] = Vector_IntersectPlane(plane_p, plane_n, *inside_points[0], *outside_points[0]);
        out_tri1.p[2] = Vector_IntersectPlane(plane_p, plane_n, *inside_points[0], *outside_points[1]);

        return 1; // Return the newly formed single triangle
    }

    if (nInsidePointCount == 2 && nOutsidePointCount == 1)
    {
        // Triangle should be clipped. As two points lie inside the plane,
        // the clipped triangle becomes a "quad". Fortunately, we can
        // represent a quad with two new triangles

        // Copy appearance info to new triangles
        out_tri1.col = in_tri.col;
        out_tri1.sym = in_tri.sym;

        out_tri2.col = in_tri.col;
        out_tri2.sym = in_tri.sym;

        // The first triangle consists of the two inside points and a new
        // point determined by the location where one side of the triangle
        // intersects with the plane
        out_tri1.p[0] = *inside_points[0];
        out_tri1.p[1] = *inside_points[1];
        out_tri1.p[2] = Vector_IntersectPlane(plane_p, plane_n, *inside_points[0], *outside_points[0]);

        // The second triangle is composed of one of he inside points, a
        // new point determined by the intersection of the other side of the 
        // triangle and the plane, and the newly created point above
        out_tri2.p[0] = *inside_points[1];
        out_tri2.p[1] = out_tri1.p[2];
        out_tri2.p[2] = Vector_IntersectPlane(plane_p, plane_n, *inside_points[1], *outside_points[0]);

        return 2; // Return two newly formed triangles which form a quad
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
    texGrass.loadFromFile("assets/textures/grass.png");
    texTop.loadFromFile("assets/textures/top.png");
    texSide.loadFromFile("assets/textures/side.png");
    texBottom.loadFromFile("assets/textures/bottom.png");


    onUserCreate(*pWnd);

}

SFML3D::~SFML3D()
{
}

bool SFML3D::onUserCreate(sf::RenderWindow& wnd_)
{
    pWnd = &wnd_;


    cubeMesh.tris = 
    //cubeMesh.LoadFromObjectFile("models/axis.cid");
    {
        { 0.f, 0.f, 0.f, 1.f,   0.f, 1.f, 0.f, 1.f,    1.f, 1.f, 0.f, 1.f},
        {0.f, 0.f, 0.f, 1.f,   1.f, 1.f, 0.f, 1.f,    1.f, 0.f, 0.f, 1.f}, // FRONT

        { 1.f, 0.f, 1.f, 1.f,    1.f, 1.f, 1.f, 1.f,    0.f, 1.f, 1.f, 1.f },
        {1.f, 0.f, 1.f, 1.f,   0.f, 1.f, 1.f, 1.f,    0.f, 0.f, 1.f, 1.f}, // BACK

        {1.f,0.f,0.f, 1.f,   1.f,1.f,0.f, 1.f,   1.f,1.f,1.f, 1.f },
        {1.f,0.f,0.f, 1.f,   1.f,1.f,1.f, 1.f,   1.f,0.f,1.f, 1.f},  // East
        {0.f,0.f,1.f, 1.f,   0.f,1.f,1.f, 1.f,   0.f,1.f,0.f, 1.f },
        {0.f,0.f,1.f, 1.f,   0.f,1.f,0.f, 1.f,   0.f,0.f,0.f, 1.f},  // West
        {0.f,1.f,0.f, 1.f,   0.f,1.f,1.f, 1.f,   1.f,1.f,1.f, 1.f },
        {0.f,1.f,0.f, 1.f,   1.f,1.f,1.f, 1.f,   1.f,1.f,0.f, 1.f}, // Top
        {0.f,0.f,1.f, 1.f,   0.f,0.f,0.f, 1.f,   1.f,0.f,0.f, 1.f },
        {0.f,0.f,1.f, 1.f,   1.f,0.f,0.f, 1.f,   1.f,0.f,1.f, 1.f} // Bottom
    };

    cubeMesh.faceTypes =
    {
        FaceType::Side,
        FaceType::Side,
        FaceType::Side,
        FaceType::Side,
        FaceType::Top,
        FaceType::Bottom
    };

    matProj = Matrix_MakeProjection(45.f, (float)WH / (float)WW, 0.1f, 1000.f);

    /*float fNear = 0.1f;
    float fFar = 1000.f;
    float fFov = 87.f;
    float fAspectRatio = (float)WH / (float)WW;
    float fFovRad = 1.f / (float)(std::tanf(fFov * 0.5f / 180.f * (float)M_PI));

    matProj.m[0][0] = fAspectRatio * fFovRad;
    matProj.m[1][1] = fFovRad;
    matProj.m[2][2] = fFar / (fFar - fNear);
    matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
    matProj.m[2][3] = 1.f;
    matProj.m[3][3] = 0.f;*/

    return true;
}

bool SFML3D::onUserUpdate(float elapsedTime)
{

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up))
        vCam.y += 8.0f * fElapsedTime;	// Travel Upwards

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down))
        vCam.y -= 8.0f * fElapsedTime;	// Travel Downwards


    // Dont use these two in FPS mode, it is confusing :P
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left))
        vCam.x -= 8.0f * fElapsedTime;	// Travel Along X-Axis

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right))
        vCam.x += 8.0f * fElapsedTime;	// Travel Along X-Axis
    ///////


    v3d vForward = Vector_Mul(vLookDir, 8.0f * fElapsedTime);


    // Standard FPS Control scheme, but turn instead of strafe
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W))
        vCam = Vector_Add(vCam, vForward);

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S))
        vCam = Vector_Sub(vCam, vForward);

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A))
        fYaw -= 2.0f * fElapsedTime;

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D))
        fYaw += 2.0f * fElapsedTime;

    

   
    fElapsedTime = timer.restart().asSeconds();
    Mat4x4 matRotZ, matRotX;
   // fTheta += 1.f * fElapsedTime;
    if (fTheta >= 360.f)
    { fTheta = 0.f; }
    // RotationZ
    matRotZ = Matrix_MakeRotationZ(fTheta * 0.5f);
    matRotX = Matrix_MakeRotationX(fTheta);

    Mat4x4 matTrans;
    matTrans = Matrix_MakeTranslation(0.f, 0.f, 16.f);

    Mat4x4 matWorld;
    matWorld = Matrix_MakeIdentity();
    matWorld = Matrix_MultiplyMatrix(matRotZ, matRotX);
    matWorld = Matrix_MultiplyMatrix(matWorld, matTrans);

    v3d vUp = { 0.f,1.f,0.f };
    v3d vTarget = { 0.f,0.f,1.f };
    Mat4x4 matCameraRot = Matrix_MakeRotationY(fYaw);
    vLookDir = Matrix_MultiplyVector(matCameraRot, vTarget);
    vTarget = Vector_Add(vCam, vLookDir);

    Mat4x4 matCamera = Matrix_PointAt(vCam, vTarget, vUp);

    Mat4x4 matView = Matrix_QuickInverse(matCamera);

    std::vector<Tri3D> vecTrianglesToRaster;


    for (int i = 0; i < 4; i++)
    {
        cubeMesh.faceTypes.push_back(FaceType::Side);
    }

    for (int i = 0; i < 1; i++)
    {
        cubeMesh.faceTypes.push_back(FaceType::Top);
    }


    for (int i = 0; i < 1; i++)
    {
        cubeMesh.faceTypes.push_back(FaceType::Bottom);
    }

    cubeMesh.tris[0].faceType = FaceType::Side;
    cubeMesh.tris[1].faceType = FaceType::Side;
    cubeMesh.tris[2].faceType = FaceType::Side;
    cubeMesh.tris[3].faceType = FaceType::Side;

    cubeMesh.tris[4].faceType = FaceType::Side;
    cubeMesh.tris[5].faceType = FaceType::Side;
    cubeMesh.tris[6].faceType = FaceType::Side;
    cubeMesh.tris[7].faceType = FaceType::Side;

    cubeMesh.tris[8].faceType = FaceType::Top;
    cubeMesh.tris[9].faceType = FaceType::Top;

    cubeMesh.tris[10].faceType = FaceType::Bottom;
    cubeMesh.tris[11].faceType = FaceType::Bottom;


   

    for (auto& tri : cubeMesh.tris)
    {


        Tri3D triProjected, triTransformed, triViewed;

        triTransformed.p[0] = Matrix_MultiplyVector(matWorld, tri.p[0]);
        triTransformed.p[1] = Matrix_MultiplyVector(matWorld, tri.p[1]);
        triTransformed.p[2] = Matrix_MultiplyVector(matWorld, tri.p[2]);
        triTransformed.faceType = tri.faceType;


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


        v3d normal, line1, line2, start;

        line1 = Vector_Sub(triTransformed.p[1], triTransformed.p[0]);
        line2 = Vector_Sub(triTransformed.p[2], triTransformed.p[0]);
        start = { triTransformed.p[0] };
        
        Line3D l1{ line1.x, line1.y, line1.z };
        Line3D l2{ line2.x, line2.y, line2.z };

 //       Line3D line2{ triTranslated.p[2].x - triTranslated.p[0].x, triTranslated.p[2].y - triTranslated.p[0].y, triTranslated.p[2].z - triTranslated.p[0].z };

        normal = crossProd(l1, l2);
        normal = Vector_Normalise(normal);

       /* float l = sqrtf(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
        normal.x /= 1.f; normal.y /= 1.f; normal.z /= 1.f;*/
        
        v3d vCameraRay = Vector_Sub(triTransformed.p[0], vCam);

      /*  if (normal.z < 0)*/
        if(Vector_DotProduct(normal, vCameraRay) < 0.f)
        {
            // Illumination
            v3d lightDirection = { 0.f, 1.f, -1.f };
            lightDirection = Vector_Normalise(lightDirection);/*
            float l = sqrtf(lightDirection.x * lightDirection.x + lightDirection.y * lightDirection.y + lightDirection.z * lightDirection.z);
            lightDirection.x /= 1.f; lightDirection.y /= 1.f; lightDirection.z /= 1.f;*/

            float dp = std::max(0.1f, Vector_DotProduct(lightDirection, normal));

            COLOR c = getColor(dp);
            triTransformed.col = c.bgCol;
            triTransformed.sym = ShadeStyle::PIXEL_SOLID;

            triViewed.p[0] = Matrix_MultiplyVector(matView, triTransformed.p[0]);
            triViewed.p[1] = Matrix_MultiplyVector(matView, triTransformed.p[1]);
            triViewed.p[2] = Matrix_MultiplyVector(matView, triTransformed.p[2]);
            triViewed.col = triTransformed.col;
            triViewed.sym = triTransformed.sym;
            triViewed.faceType = triTransformed.faceType;


            // Clip Viewed Triangle against near plane, this could form two additional
                // additional triangles. 
            int nClippedTriangles = 0;
            Tri3D clipped[2];
            nClippedTriangles = Triangle_ClipAgainstPlane({ 0.0f, 0.0f, 0.1f }, { 0.0f, 0.0f, 1.0f }, triViewed, clipped[0], clipped[1]);
            clipped[0].faceType = triViewed.faceType;
            clipped[1].faceType = triViewed.faceType;


            for (int n = 0; n < nClippedTriangles; n++)
            {
                // Project triangles from 3D --> 2D
                triProjected.p[0] = Matrix_MultiplyVector(matProj, clipped[n].p[0]);
                triProjected.p[1] = Matrix_MultiplyVector(matProj, clipped[n].p[1]);
                triProjected.p[2] = Matrix_MultiplyVector(matProj, clipped[n].p[2]);
                triProjected.col = clipped[n].col;
                triProjected.sym = clipped[n].sym;
                triProjected.faceType = clipped[n].faceType;


                // Scale into view, we moved the normalising into cartesian space
                // out of the matrix.vector function from the previous videos, so
                // do this manually
                triProjected.p[0] = Vector_Div(triProjected.p[0], triProjected.p[0].w);
                triProjected.p[1] = Vector_Div(triProjected.p[1], triProjected.p[1].w);
                triProjected.p[2] = Vector_Div(triProjected.p[2], triProjected.p[2].w);

                // X/Y are inverted so put them back
                triProjected.p[0].x *= -1.0f;
                triProjected.p[1].x *= -1.0f;
                triProjected.p[2].x *= -1.0f;
                triProjected.p[0].y *= -1.0f;
                triProjected.p[1].y *= -1.0f;
                triProjected.p[2].y *= -1.0f;

                // Offset verts into visible normalised space
                v3d vOffsetView = { 1,1,0 };
                triProjected.p[0] = Vector_Add(triProjected.p[0], vOffsetView);
                triProjected.p[1] = Vector_Add(triProjected.p[1], vOffsetView);
                triProjected.p[2] = Vector_Add(triProjected.p[2], vOffsetView);
                triProjected.p[0].x *= 0.5f * (float)WW;
                triProjected.p[0].y *= 0.5f * (float)WH;
                triProjected.p[1].x *= 0.5f * (float)WW;
                triProjected.p[1].y *= 0.5f * (float)WH;
                triProjected.p[2].x *= 0.5f * (float)WW;
                triProjected.p[2].y *= 0.5f * (float)WH;

                // Store triangle for sorting
                vecTrianglesToRaster.push_back(triProjected);
            }

           // triProjected.p[0] = Matrix_MultiplyVector(matProj, triViewed.p[0]);
           // triProjected.p[1] = Matrix_MultiplyVector(matProj, triViewed.p[1]);
           // triProjected.p[2] = Matrix_MultiplyVector(matProj, triViewed.p[2]);
           // triProjected.col = triTransformed.col;
           // triProjected.sym = triTransformed.sym;


           // triProjected.p[0] = Vector_Div(triProjected.p[0], triProjected.p[0].w);
           // triProjected.p[1] = Vector_Div(triProjected.p[1], triProjected.p[1].w);
           // triProjected.p[2] = Vector_Div(triProjected.p[2], triProjected.p[2].w);


           // //MulMatVec(triTranslated.p[1], triProjected.p[1], matProj);
           // //MulMatVec(triTranslated.p[2], triProjected.p[2], matProj);

           // //triProjected.col = triTranslated.col;
           // //triProjected.sym = triTranslated.sym;


           ///* triProjected.p[0].x += 1.f; triProjected.p[0].y += 1.f;
           // triProjected.p[1].x += 1.f; triProjected.p[1].y += 1.f;
           // triProjected.p[2].x += 1.f; triProjected.p[2].y += 1.f;*/

           // v3d vOffsetView = { 1.f,1.f,0.f };
           // triProjected.p[0] = Vector_Add(triProjected.p[0], vOffsetView);
           // triProjected.p[1] = Vector_Add(triProjected.p[1], vOffsetView);
           // triProjected.p[2] = Vector_Add(triProjected.p[2], vOffsetView);


           // triProjected.p[0].x *= 0.5f * (float)WW;
           // triProjected.p[0].y *= 0.5f * (float)WH;
           // triProjected.p[1].x *= 0.5f * (float)WW;
           // triProjected.p[1].y *= 0.5f * (float)WH;
           // triProjected.p[2].x *= 0.5f * (float)WW;
           // triProjected.p[2].y *= 0.5f * (float)WH;



           // // triProjected.p[0].x = (triProjected.p[0].x * sizex);
           // // triProjected.p[0].y = (triProjected.p[0].y * -sizey);
           // //// triProjected.p[0].z = triRotateZX.p[0].z + 3.f;
           // // triProjected.p[1].x = (triProjected.p[1].x * sizex);
           // // triProjected.p[1].y = (triProjected.p[1].y * -sizey);
           // //// triProjected.p[1].z = triRotateZX.p[1].z + 3.f;
           // // triProjected.p[2].x = (triProjected.p[2].x * sizex);
           // // triProjected.p[2].y = (triProjected.p[2].y * -sizey);
           ////  triProjected.p[2].z = triRotateZX.p[2].z + 3.f;

           // vecTrianglesToRaster.push_back(triProjected);


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


    pWnd->clear(sf::Color(sf::Color::Blue));

    for (auto& triToRaster : vecTrianglesToRaster)
    {
        // Clip triangles against all four screen edges, this could yield
        // a bunch of triangles, so create a queue that we traverse to 
        //  ensure we only test new triangles generated against planes
        Tri3D clipped[2];
        std::list<Tri3D> listTriangles;

        // Add initial triangle
        listTriangles.push_back(triToRaster);
        int nNewTriangles = 1;

        for (int p = 0; p < 4; p++)
        {
            int nTrisToAdd = 0;
            while (nNewTriangles > 0)
            {
                // Take triangle from front of queue
                Tri3D test = listTriangles.front();
                listTriangles.pop_front();
                nNewTriangles--;

                // Clip it against a plane. We only need to test each 
                // subsequent plane, against subsequent new triangles
                // as all triangles after a plane clip are guaranteed
                // to lie on the inside of the plane. I like how this
                // comment is almost completely and utterly justified
                switch (p)
                {
                case 0:	nTrisToAdd = Triangle_ClipAgainstPlane({ 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, test, clipped[0], clipped[1]); break;
                case 1:	nTrisToAdd = Triangle_ClipAgainstPlane({ 0.0f, (float)WH - 1, 0.0f }, { 0.0f, -1.0f, 0.0f }, test, clipped[0], clipped[1]); break;
                case 2:	nTrisToAdd = Triangle_ClipAgainstPlane({ 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, test, clipped[0], clipped[1]); break;
                case 3:	nTrisToAdd = Triangle_ClipAgainstPlane({ (float)WW - 1, 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f }, test, clipped[0], clipped[1]); break;
                }

                // Clipping may yield a variable number of triangles, so
                // add these new ones to the back of the queue for subsequent
                // clipping against next planes
                for (int w = 0; w < nTrisToAdd; w++)
                    listTriangles.push_back(clipped[w]);
            }
            nNewTriangles = listTriangles.size();
        }


        // Draw the transformed, viewed, clipped, projected, sorted, clipped triangles
        for (auto& t : listTriangles)
        {

            Tri2D tri2d{ {t.p[0].x, t.p[0].y},{t.p[1].x,t.p[1].y}, {t.p[2].x, t.p[2].y} };
            tri2d.faceType = t.faceType;
            // Tri2D tri2d{ {posx + (0.f * sizex), posy + (0.f * -sizey)},{posx + (0.5f * sizex), posy + (1.f * sizey)}, {posx + (1.f * sizex), posy + (0.f * sizey)} };

            drawTriangle(tri2d, *pWnd, t.col, drawBoth);

            //FillTriangle(t.p[0].x, t.p[0].y, t.p[1].x, t.p[1].y, t.p[2].x, t.p[2].y, t.sym, t.col);
            //DrawTriangle(t.p[0].x, t.p[0].y, t.p[1].x, t.p[1].y, t.p[2].x, t.p[2].y, PIXEL_SOLID, FG_BLACK);
        }
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
