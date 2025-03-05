#include "MachineObject.cuh"

CollisionObject::CollisionObject()
{
}

CollisionObject::CollisionObject(MachineType _type)
{
  type = _type;
}

CollisionObject::~CollisionObject()
{
}

std::vector<Mesh> CollisionObject::move(PrintItem item, Kinematics* kin, bool isdraw)
{
  std::vector<Mesh> ret;
  return ret;
}

CollisionCylinder::CollisionCylinder()
{
}

CollisionCylinder::CollisionCylinder(MachineType _type, tinyxml2::XMLElement* xml)
{
  type = _type;
  // input
  tinyxml2::XMLElement* xmlLowerRad = xml->FirstChildElement("LowerRadius");
  lower_radius = stof(xmlLowerRad->GetText());
  tinyxml2::XMLElement* xmlUpperRad = xml->FirstChildElement("UpperRadius");
  upper_radius = stof(xmlUpperRad->GetText());
  tinyxml2::XMLElement* xmlHeight = xml->FirstChildElement("Height");
  height = stof(xmlHeight->GetText());
  tinyxml2::XMLElement* xmlLowerX = xml->FirstChildElement("LowerX");
  lowerX = stof(xmlLowerX->GetText());
  tinyxml2::XMLElement* xmlLowerY = xml->FirstChildElement("LowerY");
  lowerY = stof(xmlLowerY->GetText());
  tinyxml2::XMLElement* xmlLowerZ = xml->FirstChildElement("LowerZ");
  lowerZ = stof(xmlLowerZ->GetText());
  tinyxml2::XMLElement* xmlDiv = xml->FirstChildElement("Div");
  int _div = stoi(xmlDiv->GetText());
  (_div > 2) ? div = _div : div = 3;

  // calc value
  vertices = new float* [div * 2];
  triNum = (div - 2) * 2 + div * 2;
  indices = new int* [triNum];
  for (int i = 0; i < div; i++)
  {
    float angle = (float)i * 2.0 * M_PI / div;
    vertices[i] = new float[3];
    vertices[i][0] = (lower_radius)*cos(angle) + lowerX;
    vertices[i][1] = (lower_radius)*sin(angle) + lowerY;
    vertices[i][2] = lowerZ;
    vertices[i + div] = new float[3];
    vertices[i + div][0] = (upper_radius)*cos(angle) + lowerX;
    vertices[i + div][1] = (upper_radius)*sin(angle) + lowerY;
    vertices[i + div][2] = lowerZ + height;
    // side
    int l1 = i;
    int l2 = i + 1;
    int u1 = i + div;
    int u2 = i + 1 + div;
    if (l2 == div)
    {
      l2 = 0;
    }
    if (u2 == div * 2)
    {
      u2 = div;
    }
    indices[i] = new int[3];
    indices[i][0] = l1;
    indices[i][1] = l2;
    indices[i][2] = u1;
    indices[i + div] = new int[3];
    indices[i + div][0] = u1;
    indices[i + div][1] = u2;
    indices[i + div][2] = l2;
    // upper and lower
    if (i > 1)
    {
      int idx_l = div * 2 + i - 2;
      indices[idx_l] = new int[3];
      indices[idx_l][0] = 0;
      indices[idx_l][1] = i - 1;
      indices[idx_l][2] = i;
      int idx_h = div * 3 + i - 4;
      indices[idx_h] = new int[3];
      indices[idx_h][0] = div;
      indices[idx_h][1] = i - 1 + div;
      indices[idx_h][2] = i + div;
    }
  }
}

CollisionCylinder::~CollisionCylinder()
{
}

std::vector<Mesh> CollisionCylinder::move(PrintItem item, Kinematics* kin, bool isdraw)
{
  float mat[3 * 3];
  std::vector<Mesh> ret;
  Mesh sub;
  float buf[3];
  float bufPos[3];
  float deltaPos[3];
  float delta[3];

  kin->GetRotationMatrix(item.edMoveVal[3], item.edMoveVal[4], item.edMoveVal[5], mat);
  float** vertMove = new float* [div * 2];
  for (int i = 0; i < div * 2; i++)
  {
    vertMove[i] = new float[3];
    switch (type)
    {
    case MachineType::Head:
      deltaPos[0] = lowerX;
      deltaPos[1] = lowerY;
      deltaPos[2] = lowerZ + height / 2.0;
      calcMatVec(mat, deltaPos, delta);
      bufPos[0] = vertices[i][0] - lowerX;
      bufPos[1] = vertices[i][1] - lowerY;
      bufPos[2] = vertices[i][2] - lowerZ - height / 2.0;
      calcMatVec(mat, bufPos, buf);
      vertMove[i][0] = buf[0] + delta[0] + item.edPnt[0];
      vertMove[i][1] = buf[1] + delta[1] + item.edPnt[1];
      vertMove[i][2] = buf[2] + delta[2] + item.edPnt[2];
      break;
    case MachineType::XGantry:
      deltaPos[0] = lowerX;
      deltaPos[1] = item.edMoveVal[1] + lowerY;
      deltaPos[2] = item.edMoveVal[2] + lowerZ + height / 2.0;
      calcMatVec(mat, deltaPos, delta);
      bufPos[0] = vertices[i][0] - lowerX;
      bufPos[1] = vertices[i][1] - lowerY;
      bufPos[2] = vertices[i][2] - lowerZ - height / 2.0;
      calcMatVec(mat, bufPos, buf);
      vertMove[i][0] = buf[0] + delta[0];
      vertMove[i][1] = buf[1] + delta[1];
      vertMove[i][2] = buf[2] + delta[2];
      break;
    case MachineType::YGantry:
      deltaPos[0] = item.edMoveVal[0] + lowerX;
      deltaPos[1] = lowerY;
      deltaPos[2] = item.edMoveVal[2] + lowerZ + height / 2.0;
      calcMatVec(mat, deltaPos, delta);
      bufPos[0] = vertices[i][0] - lowerX;
      bufPos[1] = vertices[i][1] - lowerY;
      bufPos[2] = vertices[i][2] - lowerZ - height / 2.0;
      calcMatVec(mat, bufPos, buf);
      vertMove[i][0] = buf[0] + delta[0];
      vertMove[i][1] = buf[1] + delta[1];
      vertMove[i][2] = buf[2] + delta[2];
      break;
    case MachineType::Bed:
      float nonmat[3 * 3];
      kin->GetRotationMatrix(0.0, 0.0, 0.0, nonmat);
      deltaPos[0] = lowerX;
      deltaPos[1] = lowerY;
      deltaPos[2] = lowerZ + height / 2.0;
      calcMatVec(mat, deltaPos, delta);
      bufPos[0] = vertices[i][0] - lowerX;
      bufPos[1] = vertices[i][1] - lowerY;
      bufPos[2] = vertices[i][2] - lowerZ - height / 2.0;
      calcMatVec(nonmat, bufPos, buf);
      vertMove[i][0] = buf[0] + delta[0];
      vertMove[i][1] = buf[1] + delta[1];
      vertMove[i][2] = buf[2] + delta[2];
      break;
    }
  }
  for (int i = 0; i < triNum; i++)
  {
    sub.pnt0[0] = vertMove[indices[i][0]][0];
    sub.pnt0[1] = vertMove[indices[i][0]][1];
    sub.pnt0[2] = vertMove[indices[i][0]][2];
    sub.pnt1[0] = vertMove[indices[i][1]][0];
    sub.pnt1[1] = vertMove[indices[i][1]][1];
    sub.pnt1[2] = vertMove[indices[i][1]][2];
    sub.pnt2[0] = vertMove[indices[i][2]][0];
    sub.pnt2[1] = vertMove[indices[i][2]][1];
    sub.pnt2[2] = vertMove[indices[i][2]][2];
    sub.id = INT_MAX;
    ret.push_back(sub);
  }

  // draw openGl myself
  if (isdraw)
  {
    glPushMatrix();
    switch (type)
    {
    case MachineType::Head:
      glColor4d(HEAD_FACE_COLOR_R, HEAD_FACE_COLOR_G, HEAD_FACE_COLOR_B, HEAD_FACE_TRANS);
      break;
    case MachineType::XGantry:
      glColor4d(GANTRY_FACE_COLOR_R, GANTRY_FACE_COLOR_G, GANTRY_FACE_COLOR_B, GANTRY_FACE_TRANS);
      break;
    case MachineType::YGantry:
      glColor4d(GANTRY_FACE_COLOR_R, GANTRY_FACE_COLOR_G, GANTRY_FACE_COLOR_B, GANTRY_FACE_TRANS);
      break;
    case MachineType::Bed:
      glColor4d(BED_FACE_COLOR_R, BED_FACE_COLOR_G, BED_FACE_COLOR_B, BED_FACE_TRANS);
      break;
    }
    for (int i = 0; i < triNum; i++)
    {
      glBegin(GL_POLYGON);
      glVertex3f(ret[i].pnt0[0], ret[i].pnt0[1], ret[i].pnt0[2]);
      glVertex3f(ret[i].pnt1[0], ret[i].pnt1[1], ret[i].pnt1[2]);
      glVertex3f(ret[i].pnt2[0], ret[i].pnt2[1], ret[i].pnt2[2]);
      glEnd();
    }
    switch (type)
    {
    case MachineType::Head:
      glColor4d(HEAD_EDGE_COLOR_R, HEAD_EDGE_COLOR_G, HEAD_EDGE_COLOR_B, HEAD_EDGE_TRANS);
      break;
    case MachineType::XGantry:
      glColor4d(GANTRY_EDGE_COLOR_R, GANTRY_EDGE_COLOR_G, GANTRY_EDGE_COLOR_B, GANTRY_EDGE_TRANS);
      break;
    case MachineType::YGantry:
      glColor4d(GANTRY_EDGE_COLOR_R, GANTRY_EDGE_COLOR_G, GANTRY_EDGE_COLOR_B, GANTRY_EDGE_TRANS);
      break;
    case MachineType::Bed:
      glColor4d(BED_EDGE_COLOR_R, BED_EDGE_COLOR_G, BED_EDGE_COLOR_B, BED_EDGE_TRANS);
      break;
    }
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < div; i++)
    {
      glVertex3f(vertMove[i][0], vertMove[i][1], vertMove[i][2]);
    }
    glEnd();
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < div; i++)
    {
      glVertex3f(vertMove[i + div][0], vertMove[i + div][1], vertMove[i + div][2]);
    }
    glEnd();
    glPopMatrix();
  }

  return ret;
}

CollisionBox::CollisionBox()
{
}

CollisionBox::CollisionBox(MachineType _type, tinyxml2::XMLElement* xml)
{
  type = _type;
  // input
  tinyxml2::XMLElement* xmlCenterX = xml->FirstChildElement("CenterX");
  centerX = stof(xmlCenterX->GetText());
  tinyxml2::XMLElement* xmlCenterY = xml->FirstChildElement("CenterY");
  centerY = stof(xmlCenterY->GetText());
  tinyxml2::XMLElement* xmlCenterZ = xml->FirstChildElement("CenterZ");
  centerZ = stof(xmlCenterZ->GetText());
  tinyxml2::XMLElement* xmlSizeX = xml->FirstChildElement("SizeX");
  sizeX = stof(xmlSizeX->GetText());
  tinyxml2::XMLElement* xmlSizeY = xml->FirstChildElement("SizeY");
  sizeY = stof(xmlSizeY->GetText());
  tinyxml2::XMLElement* xmlSizeZ = xml->FirstChildElement("SizeZ");
  sizeZ = stof(xmlSizeZ->GetText());

  // calc value
  vertices = new float* [8];
  indices = new int* [12];
  float basePnt[8][3] = {
    {-sizeX / 2.0, sizeY / 2.0, -sizeZ / 2.0},
    {sizeX / 2.0, sizeY / 2.0, -sizeZ / 2.0},
    {sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0},
    {-sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0},
    {-sizeX / 2.0, -sizeY / 2.0, -sizeZ / 2.0},
    {-sizeX / 2.0, -sizeY / 2.0, sizeZ / 2.0},
    {sizeX / 2.0, -sizeY / 2.0, sizeZ / 2.0},
    {sizeX / 2.0, -sizeY / 2.0, -sizeZ / 2.0}
  };
  int baseMesh[12][3] = {
    {0, 1, 2},
    {4, 5, 6},
    {0, 3, 5},
    {3, 2, 6},
    {2, 1, 7},
    {1, 7, 4},
    {0, 2, 3},
    {4, 6, 7},
    {0, 5, 4},
    {3, 6, 5},
    {2, 7, 6},
    {1, 4, 0}
  };
  for (int i = 0; i < 8; i++)
  {
    vertices[i] = new float[3];
    vertices[i][0] = basePnt[i][0] + centerX;
    vertices[i][1] = basePnt[i][1] + centerY;
    vertices[i][2] = basePnt[i][2] + centerZ;
  }
  for (int i = 0; i < 12; i++)
  {
    indices[i] = new int[3];
    indices[i][0] = baseMesh[i][0];
    indices[i][1] = baseMesh[i][1];
    indices[i][2] = baseMesh[i][2];
  }
}

CollisionBox::~CollisionBox()
{
}

std::vector<Mesh> CollisionBox::move(PrintItem item, Kinematics* kin, bool isdraw)
{
  float mat[3 * 3];
  std::vector<Mesh> ret;
  Mesh sub;
  float buf[3];
  float bufPos[3];
  float deltaPos[3];
  float delta[3];

  //auto t1 = std::chrono::high_resolution_clock::now();
  kin->GetRotationMatrix(item.edMoveVal[3], item.edMoveVal[4], item.edMoveVal[5], mat);
  //auto t2 = std::chrono::high_resolution_clock::now();
  float** vertMove = new float* [8];
  for (int i = 0; i < 8; i++)
  {
    vertMove[i] = new float[3];
    switch (type)
    {
    case MachineType::Head:
      deltaPos[0] = centerX;
      deltaPos[1] = centerY;
      deltaPos[2] = centerZ;
      calcMatVec(mat, deltaPos, delta);
      bufPos[0] = vertices[i][0] - centerX;
      bufPos[1] = vertices[i][1] - centerY;
      bufPos[2] = vertices[i][2] - centerZ;
      calcMatVec(mat, bufPos, buf);
      vertMove[i][0] = buf[0] + delta[0] + item.edPnt[0];
      vertMove[i][1] = buf[1] + delta[1] + item.edPnt[1];
      vertMove[i][2] = buf[2] + delta[2] + item.edPnt[2];
      break;
    case MachineType::XGantry:
      deltaPos[0] = centerX;
      deltaPos[1] = item.edMoveVal[1] + centerY;
      deltaPos[2] = item.edMoveVal[2] + centerZ;
      calcMatVec(mat, deltaPos, delta);
      bufPos[0] = vertices[i][0] - centerX;
      bufPos[1] = vertices[i][1] - centerY;
      bufPos[2] = vertices[i][2] - centerZ;
      calcMatVec(mat, bufPos, buf);
      vertMove[i][0] = buf[0] + delta[0];
      vertMove[i][1] = buf[1] + delta[1];
      vertMove[i][2] = buf[2] + delta[2];
      break;
    case MachineType::YGantry:
      deltaPos[0] = item.edMoveVal[0] + centerX;
      deltaPos[1] = centerY;
      deltaPos[2] = item.edMoveVal[2] + centerZ;
      calcMatVec(mat, deltaPos, delta);
      bufPos[0] = vertices[i][0] - centerX;
      bufPos[1] = vertices[i][1] - centerY;
      bufPos[2] = vertices[i][2] - centerZ;
      calcMatVec(mat, bufPos, buf);
      vertMove[i][0] = buf[0] + delta[0];
      vertMove[i][1] = buf[1] + delta[1];
      vertMove[i][2] = buf[2] + delta[2];
      break;
    case MachineType::Bed:
      float nonmat[3 * 3];
      kin->GetRotationMatrix(0.0, 0.0, 0.0, nonmat);
      deltaPos[0] = centerX;
      deltaPos[1] = centerY;
      deltaPos[2] = centerZ;
      calcMatVec(mat, deltaPos, delta);
      bufPos[0] = vertices[i][0] - centerX;
      bufPos[1] = vertices[i][1] - centerY;
      bufPos[2] = vertices[i][2] - centerZ;
      calcMatVec(nonmat, bufPos, buf);
      vertMove[i][0] = buf[0] + delta[0];
      vertMove[i][1] = buf[1] + delta[1];
      vertMove[i][2] = buf[2] + delta[2];
      break;
    }
  }
  for (int i = 0; i < 12; i++)
  {
    sub.pnt0[0] = vertMove[indices[i][0]][0];
    sub.pnt0[1] = vertMove[indices[i][0]][1];
    sub.pnt0[2] = vertMove[indices[i][0]][2];
    sub.pnt1[0] = vertMove[indices[i][1]][0];
    sub.pnt1[1] = vertMove[indices[i][1]][1];
    sub.pnt1[2] = vertMove[indices[i][1]][2];
    sub.pnt2[0] = vertMove[indices[i][2]][0];
    sub.pnt2[1] = vertMove[indices[i][2]][1];
    sub.pnt2[2] = vertMove[indices[i][2]][2];
    sub.id = INT_MAX;
    ret.push_back(sub);
  }
  //auto t3 = std::chrono::high_resolution_clock::now();
  //auto t12 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
  //auto t23 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
  //printf("%d,%d\n", t12.count(), t23.count());

  // draw openGl myself
  if (isdraw)
  {
    glPushMatrix();
    switch (type)
    {
    case MachineType::Head:
      glColor4d(HEAD_FACE_COLOR_R, HEAD_FACE_COLOR_G, HEAD_FACE_COLOR_B, HEAD_FACE_TRANS);
      break;
    case MachineType::XGantry:
      glColor4d(GANTRY_FACE_COLOR_R, GANTRY_FACE_COLOR_G, GANTRY_FACE_COLOR_B, GANTRY_FACE_TRANS);
      break;
    case MachineType::YGantry:
      glColor4d(GANTRY_FACE_COLOR_R, GANTRY_FACE_COLOR_G, GANTRY_FACE_COLOR_B, GANTRY_FACE_TRANS);
      break;
    case MachineType::Bed:
      glColor4d(BED_FACE_COLOR_R, BED_FACE_COLOR_G, BED_FACE_COLOR_B, BED_FACE_TRANS);
      break;
    }
    for (int i = 0; i < 12; i++)
    {
      glBegin(GL_POLYGON);
      glVertex3f(ret[i].pnt0[0], ret[i].pnt0[1], ret[i].pnt0[2]);
      glVertex3f(ret[i].pnt1[0], ret[i].pnt1[1], ret[i].pnt1[2]);
      glVertex3f(ret[i].pnt2[0], ret[i].pnt2[1], ret[i].pnt2[2]);
      glEnd();
    }
    switch (type)
    {
    case MachineType::Head:
      glColor4d(HEAD_EDGE_COLOR_R, HEAD_EDGE_COLOR_G, HEAD_EDGE_COLOR_B, HEAD_EDGE_TRANS);
      break;
    case MachineType::XGantry:
      glColor4d(GANTRY_EDGE_COLOR_R, GANTRY_EDGE_COLOR_G, GANTRY_EDGE_COLOR_B, GANTRY_EDGE_TRANS);
      break;
    case MachineType::YGantry:
      glColor4d(GANTRY_EDGE_COLOR_R, GANTRY_EDGE_COLOR_G, GANTRY_EDGE_COLOR_B, GANTRY_EDGE_TRANS);
      break;
    case MachineType::Bed:
      glColor4d(BED_EDGE_COLOR_R, BED_EDGE_COLOR_G, BED_EDGE_COLOR_B, BED_EDGE_TRANS);
      break;
    }
    glBegin(GL_LINES);
    glVertex3f(vertMove[0][0], vertMove[0][1], vertMove[0][2]);
    glVertex3f(vertMove[1][0], vertMove[1][1], vertMove[1][2]);

    glVertex3f(vertMove[1][0], vertMove[1][1], vertMove[1][2]);
    glVertex3f(vertMove[2][0], vertMove[2][1], vertMove[2][2]);

    glVertex3f(vertMove[2][0], vertMove[2][1], vertMove[2][2]);
    glVertex3f(vertMove[3][0], vertMove[3][1], vertMove[3][2]);

    glVertex3f(vertMove[3][0], vertMove[3][1], vertMove[3][2]);
    glVertex3f(vertMove[0][0], vertMove[0][1], vertMove[0][2]);

    glVertex3f(vertMove[4][0], vertMove[4][1], vertMove[4][2]);
    glVertex3f(vertMove[5][0], vertMove[5][1], vertMove[5][2]);

    glVertex3f(vertMove[5][0], vertMove[5][1], vertMove[5][2]);
    glVertex3f(vertMove[6][0], vertMove[6][1], vertMove[6][2]);

    glVertex3f(vertMove[6][0], vertMove[6][1], vertMove[6][2]);
    glVertex3f(vertMove[7][0], vertMove[7][1], vertMove[7][2]);

    glVertex3f(vertMove[7][0], vertMove[7][1], vertMove[7][2]);
    glVertex3f(vertMove[4][0], vertMove[4][1], vertMove[4][2]);

    glVertex3f(vertMove[0][0], vertMove[0][1], vertMove[0][2]);
    glVertex3f(vertMove[4][0], vertMove[4][1], vertMove[4][2]);

    glVertex3f(vertMove[3][0], vertMove[3][1], vertMove[3][2]);
    glVertex3f(vertMove[5][0], vertMove[5][1], vertMove[5][2]);

    glVertex3f(vertMove[2][0], vertMove[2][1], vertMove[2][2]);
    glVertex3f(vertMove[6][0], vertMove[6][1], vertMove[6][2]);

    glVertex3f(vertMove[1][0], vertMove[1][1], vertMove[1][2]);
    glVertex3f(vertMove[7][0], vertMove[7][1], vertMove[7][2]);
    glEnd();
    glPopMatrix();
  }

  return ret;
}

