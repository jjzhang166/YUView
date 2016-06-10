/*  YUView - YUV player with advanced analytics toolset
*   Copyright (C) 2015  Institut für Nachrichtentechnik
*                       RWTH Aachen University, GERMANY
*
*   YUView is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
*
*   YUView is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with YUView.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "frameHandler.h"

#include <QPainter>

#define FRAMEHANDLER_DEBUG_OUTPUT 0
#if FRAMEHANDLER_DEBUG_OUTPUT && !QT_NO_DEBUG
#include <QDebug>
#define DEBUG_FRAMEHANDLER qDebug
#else
#define DEBUG_FRAMEHANDLER(fmt,...) ((void)0)
#endif

// ------ Initialize the static list of frame size presets ----------

frameHandler::frameSizePresetList::frameSizePresetList()
{
  names << "Custom Size" << "QCIF" << "QVGA" << "WQVGA" << "CIF" << "VGA" << "WVGA" << "4CIF" << "ITU R.BT601" << "720i/p" << "1080i/p" << "4k" << "XGA" << "XGA+";
  sizes << QSize(-1,-1) << QSize(176,144) << QSize(320, 240) << QSize(416, 240) << QSize(352, 288) << QSize(640, 480) << QSize(832, 480) << QSize(704, 576) << QSize(720, 576) << QSize(1280, 720) << QSize(1920, 1080) << QSize(3840, 2160) << QSize(1024, 768) << QSize(1280, 960);
}

/* Get all the names of the preset frame sizes in the form "Name (xxx,yyy)" in a QStringList.
 * This can be used to directly fill the combo box.
 */
QStringList frameHandler::frameSizePresetList::getFormatedNames()
{
  QStringList presetList;
  presetList.append( "Custom Size" );

  for (int i = 1; i < names.count(); i++)
  {
    QString str = QString("%1 (%2,%3)").arg( names[i] ).arg( sizes[i].width() ).arg( sizes[i].height() );
    presetList.append( str );
  }

  return presetList;
}

// Initialize the static list of frame size presets
frameHandler::frameSizePresetList frameHandler::presetFrameSizes;

// ---------------- frameHandler ---------------------------------

frameHandler::frameHandler() : ui(new Ui::frameHandler)
{
  controlsCreated = false;
  renderSkybox = false;
  texture2D = NULL;
}

frameHandler::~frameHandler()
{
  delete ui;
  if (texture2D)
  {
    texture2D->destroy();
    delete texture2D;
  }
}

QLayout *frameHandler::createFrameHandlerControls(QWidget *parentWidget, bool isSizeFixed)
{
  // Absolutely always only call this function once!
  assert(!controlsCreated);

  ui->setupUi(parentWidget);

  // Set default values
  ui->widthSpinBox->setMaximum(100000);
  ui->widthSpinBox->setValue( frameSize.width() );
  ui->widthSpinBox->setEnabled( !isSizeFixed );
  ui->heightSpinBox->setMaximum(100000);
  ui->heightSpinBox->setValue( frameSize.height() );
  ui->heightSpinBox->setEnabled( !isSizeFixed );
  ui->frameSizeComboBox->addItems( presetFrameSizes.getFormatedNames() );
  int idx = presetFrameSizes.findSize( frameSize );
  ui->frameSizeComboBox->setCurrentIndex(idx);
  ui->frameSizeComboBox->setEnabled( !isSizeFixed );
  ui->renderInSkyBox->setChecked(renderSkybox);

  // Connect all the change signals from the controls to "connectWidgetSignals()"
  connect(ui->widthSpinBox, SIGNAL(valueChanged(int)), this, SLOT(slotVideoControlChanged()));
  connect(ui->heightSpinBox, SIGNAL(valueChanged(int)), this, SLOT(slotVideoControlChanged()));
  connect(ui->frameSizeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(slotVideoControlChanged()));
  connect(ui->renderInSkyBox, SIGNAL(toggled(bool)), this, SLOT(slotSkyboxRenderingToggled(bool)));

  // The controls have been created and can be used now
  controlsCreated = true;

  return ui->frameHandlerLayout;
}

void frameHandler::setFrameSize(QSize newSize, bool emitSignal)
{
  if (newSize == frameSize)
    // Nothing to update
    return;

  // Set the new size
  cachingFrameSizeMutex.lock();
  frameSize = newSize;
  cachingFrameSizeMutex.unlock();

  if (!controlsCreated)
    // spin boxes not created yet
    return;

  // Set the width/height spin boxes without emitting another signal (disconnect/set/reconnect)
  if (!emitSignal)
  {
    QObject::disconnect(ui->widthSpinBox, SIGNAL(valueChanged(int)), NULL, NULL);
    QObject::disconnect(ui->heightSpinBox, SIGNAL(valueChanged(int)), NULL, NULL);
  }

  ui->widthSpinBox->setValue( newSize.width() );
  ui->heightSpinBox->setValue( newSize.height() );

  if (!emitSignal)
  {
    QObject::connect(ui->widthSpinBox, SIGNAL(valueChanged(int)), this, SLOT(slotVideoControlChanged()));
    QObject::connect(ui->heightSpinBox, SIGNAL(valueChanged(int)), this, SLOT(slotVideoControlChanged()));
  }
}

void frameHandler::loadCurrentImageFromFile(QString filePath)
{
  QImage input = QImage(filePath).convertToFormat(QImage::Format_RGB888);
  currentFrame = QPixmap::fromImage(input);
  
  setFrameSize(input.size());
  skyBox.loadTextureFromCubeMap(input);
  
  if (texture2D == NULL)
  {
    texture2D = new QOpenGLTexture(input, QOpenGLTexture::DontGenerateMipMaps);
    texture2D->setWrapMode(QOpenGLTexture::ClampToBorder);
    texture2D->setMagnificationFilter(QOpenGLTexture::Nearest);
  }
}

QRgb frameHandler::getPixelVal(QPoint pixelPos) 
{ 
  if (currentImage.isNull())
    currentImage = currentFrame.toImage();

  return currentImage.pixel(pixelPos); 
}
QRgb frameHandler::getPixelVal(int x, int y)
{ 
  if (currentImage.isNull())
    currentImage = currentFrame.toImage();

  return currentImage.pixel(x, y);     
}

void frameHandler::slotVideoControlChanged()
{
  // The control that caused the slot to be called
  QObject *sender = QObject::sender();

  if (sender == ui->widthSpinBox || sender == ui->heightSpinBox)
  {
    QSize newSize = QSize( ui->widthSpinBox->value(), ui->heightSpinBox->value() );
    if (newSize != frameSize)
    {
      // Set the comboBox index without causing another signal to be emitted (disconnect/set/reconnect).
      QObject::disconnect(ui->frameSizeComboBox, SIGNAL(currentIndexChanged(int)), NULL, NULL);
      int idx = presetFrameSizes.findSize( newSize );
      ui->frameSizeComboBox->setCurrentIndex(idx);
      QObject::connect(ui->frameSizeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(slotVideoControlChanged()));
    }
  }
  else if (sender == ui->frameSizeComboBox)
  {
    QSize newSize = presetFrameSizes.getSize( ui->frameSizeComboBox->currentIndex() );
    if (newSize != frameSize && newSize != QSize(-1,-1))
    {
      // Set the new size and update the controls.
      setFrameSize(newSize);    
    }
  }
}

void frameHandler::drawFrame(QPainter *painter, int frameIdx, double zoomFactor, const QMatrix4x4 &modelViewProjectionMatrix)
{
  Q_UNUSED(frameIdx);

  if (renderSkybox)
  {
    // Render the sky box
    DEBUG_FRAMEHANDLER("frameHandler::drawFrame using skyBox");
    painter->beginNativePainting();
    skyBox.renderSkyBox(modelViewProjectionMatrix);
    painter->endNativePainting();
  }
  else
  {
    if (texture2D == NULL)
      return; 

    DEBUG_FRAMEHANDLER("frameHandler::drawFrame 2D-3D");
    painter->beginNativePainting();

    glEnable(GL_TEXTURE_2D); // you should use shader, but for an example fixed pipeline is ok ;)
    
    texture2D->bind();
    glBegin(GL_TRIANGLE_STRIP);  // draw something with the texture on
    glTexCoord2f(0.0, 0.0);
    glVertex2f(-1.0 * zoomFactor, 1.0);
 
    glTexCoord2f(1.0, 0.0);
    glVertex2f(1.0 * zoomFactor, 1.0 * zoomFactor);
 
    glTexCoord2f(0.0, 1.0);
    glVertex2f(-1.0 * zoomFactor, -1.0 * zoomFactor);
 
    glTexCoord2f(1.0, 1.0);
    glVertex2f(1.0 * zoomFactor, -1.0 * zoomFactor);
    texture2D->release();
    glEnd();

    painter->endNativePainting();
  }
  //{
  //  DEBUG_FRAMEHANDLER("frameHandler::drawFrame 2D");
  //  // Create the video rect with the size of the sequence and center it.
  //  QRect videoRect;
  //  videoRect.setSize( frameSize * zoomFactor );
  //  videoRect.moveCenter( QPoint(0,0) );

  //  // Draw the current image ( currentFrame )
  //  painter->drawPixmap( videoRect, currentFrame );
  //  
  //  if (zoomFactor >= 64)
  //  {
  //    // Draw the pixel values onto the pixels
  //    drawPixelValues(painter, videoRect, zoomFactor);
  //  }
  //}
}

void frameHandler::drawPixelValues(QPainter *painter, QRect videoRect, double zoomFactor, frameHandler *item2)
{
  // Draw the pixel values onto the pixels

  // TODO: Does this also work for sequences with width/height non divisible by 2? Not sure about that.
    
  // First determine which pixels from this item are actually visible, because we only have to draw the pixel values
  // of the pixels that are actually visible
  QRect viewport = painter->viewport();
  QTransform worldTransform = painter->worldTransform();
    
  int xMin = (videoRect.width() / 2 - worldTransform.dx()) / zoomFactor;
  int yMin = (videoRect.height() / 2 - worldTransform.dy()) / zoomFactor;
  int xMax = (videoRect.width() / 2 - (worldTransform.dx() - viewport.width() )) / zoomFactor;
  int yMax = (videoRect.height() / 2 - (worldTransform.dy() - viewport.height() )) / zoomFactor;

  // Clip the min/max visible pixel values to the size of the item (no pixels outside of the
  // item have to be labeled)
  xMin = clip(xMin, 0, frameSize.width()-1);
  yMin = clip(yMin, 0, frameSize.height()-1);
  xMax = clip(xMax, 0, frameSize.width()-1);
  yMax = clip(yMax, 0, frameSize.height()-1);

  // The center point of the pixel (0,0).
  QPoint centerPointZero = ( QPoint(-frameSize.width(), -frameSize.height()) * zoomFactor + QPoint(zoomFactor,zoomFactor) ) / 2;
  // This rect has the size of one pixel and is moved on top of each pixel to draw the text
  QRect pixelRect;
  pixelRect.setSize( QSize(zoomFactor, zoomFactor) );
  for (int x = xMin; x <= xMax; x++)
  {
    for (int y = yMin; y <= yMax; y++)
    {
      // Calculate the center point of the pixel. (Each pixel is of size (zoomFactor,zoomFactor)) and move the pixelRect to that point.
      QPoint pixCenter = centerPointZero + QPoint(x * zoomFactor, y * zoomFactor);
      pixelRect.moveCenter(pixCenter);
     
      // Get the text to show
      QRgb pixVal;
      if (item2 != NULL)
      {
        QRgb pixel1 = getPixelVal(x, y);
        QRgb pixel2 = item2->getPixelVal(x, y);

        int dR = qRed(pixel1) - qRed(pixel2);
        int dG = qGreen(pixel1) - qGreen(pixel2);
        int dB = qBlue(pixel1) - qBlue(pixel2);

        int r = clip( 128 + dR, 0, 255);
        int g = clip( 128 + dG, 0, 255);
        int b = clip( 128 + dB, 0, 255);

        pixVal = qRgb(r,g,b);
      }
      else
        pixVal = getPixelVal(x, y);
      QString valText = QString("R%1\nG%2\nB%3").arg(qRed(pixVal)).arg(qGreen(pixVal)).arg(qBlue(pixVal));
           
      painter->setPen( (qRed(pixVal) < 128 && qGreen(pixVal) < 128 && qBlue(pixVal) < 128) ? Qt::white : Qt::black );
      painter->drawText(pixelRect, Qt::AlignCenter, valText);
    }
  }
}

QPixmap frameHandler::calculateDifference(frameHandler *item2, int frame, QList<infoItem> &differenceInfoList, int amplificationFactor, bool markDifference)
{
  Q_UNUSED(frame);

  int width  = qMin(frameSize.width(), item2->frameSize.width());
  int height = qMin(frameSize.height(), item2->frameSize.height());

  QImage diffImg(width, height, QImage::Format_RGB32);

  // Also calculate the MSE while we're at it (R,G,B)
  qint64 mseAdd[3] = {0, 0, 0};

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      QRgb pixel1 = getPixelVal(x, y);
      QRgb pixel2 = item2->getPixelVal(x, y);

      int dR = qRed(pixel1) - qRed(pixel2);
      int dG = qGreen(pixel1) - qGreen(pixel2);
      int dB = qBlue(pixel1) - qBlue(pixel2);

      int r, g, b;
      if (markDifference)
      {
        r = (dR != 0) ? 255 : 0;
        g = (dG != 0) ? 255 : 0;
        b = (dB != 0) ? 255 : 0;
      }
      else if (amplificationFactor != 1)
      {  
        r = clip( 128 + dR * amplificationFactor, 0, 255);
        g = clip( 128 + dG * amplificationFactor, 0, 255);
        b = clip( 128 + dB * amplificationFactor, 0, 255);
      }
      else
      {  
        r = clip( 128 + dR, 0, 255);
        g = clip( 128 + dG, 0, 255);
        b = clip( 128 + dB, 0, 255);
      }
      
      mseAdd[0] += dR * dR;
      mseAdd[1] += dG * dG;
      mseAdd[2] += dB * dB;

      QRgb val = qRgb( r, g, b );
      diffImg.setPixel(x, y, val);
    }
  }

  differenceInfoList.append( infoItem("Difference Type","RGB") );
  
  double mse[4];
  mse[0] = double(mseAdd[0]) / (width * height);
  mse[1] = double(mseAdd[1]) / (width * height);
  mse[2] = double(mseAdd[2]) / (width * height);
  mse[3] = mse[0] + mse[1] + mse[2];
  differenceInfoList.append( infoItem("MSE R",QString("%1").arg(mse[0])) );
  differenceInfoList.append( infoItem("MSE G",QString("%1").arg(mse[1])) );
  differenceInfoList.append( infoItem("MSE B",QString("%1").arg(mse[2])) );
  differenceInfoList.append( infoItem("MSE All",QString("%1").arg(mse[3])) );

  return QPixmap::fromImage(diffImg);
}

ValuePairList frameHandler::getPixelValuesDifference(QPoint pixelPos, frameHandler *item2)
{
  int width  = qMin(frameSize.width(), item2->frameSize.width());
  int height = qMin(frameSize.height(), item2->frameSize.height());

  if (pixelPos.x() < 0 || pixelPos.x() >= width || pixelPos.y() < 0 || pixelPos.y() >= height)
    return ValuePairList();

  QRgb pixel1 = getPixelVal( pixelPos );
  QRgb pixel2 = item2->getPixelVal( pixelPos );

  int r = qRed(pixel1) - qRed(pixel2);
  int g = qGreen(pixel1) - qGreen(pixel2);
  int b = qBlue(pixel1) - qBlue(pixel2);

  ValuePairList diffValues;
  diffValues.append( ValuePair("R", QString::number(r)) );
  diffValues.append( ValuePair("G", QString::number(g)) );
  diffValues.append( ValuePair("B", QString::number(b)) );
  
  return diffValues;
}

bool frameHandler::isPixelDark(QPoint pixelPos)
{
  QRgb pixVal = getPixelVal(pixelPos);
  return (qRed(pixVal) < 128 && qGreen(pixVal) < 128 && qBlue(pixVal) < 128);
}

ValuePairList frameHandler::getPixelValues(QPoint pixelPos)
{
  // Get the RGB values from the pixmap
  if (!isFormatValid())
    return ValuePairList();

  ValuePairList values;

  QRgb val = getPixelVal(pixelPos);
  values.append( ValuePair("R", QString::number(qRed(val))) );
  values.append( ValuePair("G", QString::number(qGreen(val))) );
  values.append( ValuePair("B", QString::number(qBlue(val))) );

  return values;
}

frameHandler::SkyBox::SkyBox()
{
  initializeOpenGLFunctions();

  // This will indicate that no texture is loaded yet
  textures[0] = NULL;

  // Construct a template square of size 2x2
  const QVector3D p1(-1, 1, 0); // top-left
  const QVector3D p2(-1, -1, 0); // bottom-left
  const QVector3D p3(1, -1, 0); // bottom-right
  const QVector3D p4(1, 1, 0); // top-right

  // Array for storing geometry of the cube
  QVector<QVector3D> geometry;
  geometry.reserve(24);

  // Transform p1 ... p4 for posx
  QMatrix4x4 mat;
  mat.translate(1, 0, 0);
  mat.rotate(-90, 0, 1, 0);
  geometry << mat.map(p1) << mat.map(p2) << mat.map(p3) << mat.map(p4);

  // Transform p1 ... p4 for posy
  mat.setToIdentity();
  mat.translate(0, 1, 0);
  mat.rotate(90, 1, 0, 0);
  geometry << mat.map(p1) << mat.map(p2) << mat.map(p3) << mat.map(p4);

  // Transform p2 ... p4 for posz
  mat.setToIdentity();
  mat.translate(0, 0, -1);
  geometry << mat.map(p1) << mat.map(p2) << mat.map(p3) << mat.map(p4);

  // Transform p2 ... p4 for negx
  mat.setToIdentity();
  mat.translate(-1, 0, 0);
  mat.rotate(90, 0, 1, 0);
  geometry << mat.map(p1) << mat.map(p2) << mat.map(p3) << mat.map(p4);

  // Transform p2 ... p4 for negy
  mat.setToIdentity();
  mat.translate(0, -1, 0);
  mat.rotate(-90, 1, 0, 0);
  geometry << mat.map(p1) << mat.map(p2) << mat.map(p3) << mat.map(p4);

  // Transform p2 ... p4 for negz
  mat.setToIdentity();
  mat.translate(0, 0, 1);
  mat.rotate(180, 0, 1, 0);
  geometry << mat.map(p1) << mat.map(p2) << mat.map(p3) << mat.map(p4);

  // Texture coordinates
  QVector<QVector2D> texCoords;
  texCoords.reserve(24);
  for(int i=0; i<6; i++)
    texCoords << QVector2D(0, 1) << QVector2D(0, 0) << QVector2D(1, 0) << QVector2D(1, 1);

  // Triangles
  QVector<uint> triangles;
  triangles.reserve(36);
  for(int i=0; i<6; i++)
  {
    const int base = i*4;
    triangles << base << base+1 << base+2;
    triangles << base << base+2 << base+3;
  }

  // Store the arrays in buffers
  vertexBuffer = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  vertexBuffer->create();
  vertexBuffer->bind();
  vertexBuffer->allocate( geometry.size()*sizeof(QVector3D) + texCoords.size()*sizeof(QVector2D) );
  vertexBuffer->write(0, (const void *)geometry.constData(), geometry.size()*sizeof(QVector3D) );
  vertexBuffer->write(geometry.size()*sizeof(QVector3D), (const void *)texCoords.constData(), texCoords.size()*sizeof(QVector2D) );
  vertexBuffer->release();

  indexBuffer = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  indexBuffer->create();
  indexBuffer->bind();
  indexBuffer->allocate((const void*)triangles.constData(), triangles.size()*sizeof(uint));
  indexBuffer->release();

  // Create shaders
  shader = new QOpenGLShaderProgram;
  shader->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/skybox_vertex.glsl");
  shader->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/skybox_fragment.glsl");
  shader->link();
}

frameHandler::SkyBox::~SkyBox()
{
  if (textures[0])
  {
    for (int i = 0; i < 6; i++)
      delete textures[i];
  }
}

void frameHandler::SkyBox::loadTextureFromCubeMap(QImage image)
{
  if (textures[0])
  {
    // delete the old texture
    for (int i = 0; i < 6; i++)
    {
      textures[i]->destroy();
      delete textures[i];
    }
  }

  int partWidth = image.size().width() / 4;
  int partHeight = image.size().height() / 3;

  // Layout:
  //   X
  //  XXXX
  //   X
  //// Load all texture images
  //const QImage posx = image.copy(2 * partWidth,     partHeight, partWidth, partHeight).mirrored();  // To the right
  //const QImage posy = image.copy(    partWidth,              0, partWidth, partHeight).mirrored();  // Up
  //const QImage posz = image.copy(    partWidth,     partHeight, partWidth, partHeight).mirrored();  // Straight ahead
  //const QImage negx = image.copy(            0,     partHeight, partWidth, partHeight).mirrored();  // To the left
  //const QImage negy = image.copy(    partWidth, 2 * partHeight, partWidth, partHeight).mirrored();  // bottom
  //const QImage negz = image.copy(3 * partWidth,     partHeight, partWidth, partHeight).mirrored();  // behind us

  // Load all texture images
  // Layout:
  //  X
  //  XXXX
  //  X
  const QImage posx = image.copy(    partWidth,     partHeight, partWidth, partHeight).mirrored();  // To the right
  const QImage posy = image.copy(            0,              0, partWidth, partHeight).mirrored();  // Up
  const QImage posz = image.copy(            0,     partHeight, partWidth, partHeight).mirrored();  //Straight ahead
  const QImage negx = image.copy(3 * partWidth,     partHeight, partWidth, partHeight).mirrored();  // To the left
  const QImage negy = image.copy(            0, 2 * partHeight, partWidth, partHeight).mirrored();  // bottom
  const QImage negz = image.copy(2 * partWidth,     partHeight, partWidth, partHeight).mirrored();  // behind us

  // Load images as independent texture objects
  textures[0] = new QOpenGLTexture(posx);
  textures[1] = new QOpenGLTexture(posy);
  textures[2] = new QOpenGLTexture(posz);
  textures[3] = new QOpenGLTexture(negx);
  textures[4] = new QOpenGLTexture(negy);
  textures[5] = new QOpenGLTexture(negz);
  for(int i=0; i<6; i++)
  {
    textures[i]->setWrapMode(QOpenGLTexture::ClampToEdge);
    textures[i]->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
    textures[i]->setMagnificationFilter(QOpenGLTexture::Linear);
  }
}

void frameHandler::SkyBox::renderSkyBox(const QMatrix4x4 &modelViewProjectionMatrix)
{
  if (!textures[0])
    // No texture loaded yet. Nothing to draw.
    return;

  shader->bind();
  vertexBuffer->bind();
  indexBuffer->bind();

  shader->enableAttributeArray("qt_Vertex");
  shader->setAttributeBuffer("qt_Vertex", GL_FLOAT, 0, 3, 0);

  shader->enableAttributeArray("qt_MultiTexCoord0");
  const int texCoordsOffset = 24 * sizeof(QVector3D);
  shader->setAttributeBuffer("qt_MultiTexCoord0", GL_FLOAT, texCoordsOffset, 2, 0);

  QMatrix4x4 modelMatrix = matrix;
  modelMatrix.scale(10, 10, 10);

  shader->setUniformValue("qt_ModelViewProjectionMatrix", modelViewProjectionMatrix);

  for(int i=0; i<6; i++)
 { 
    textures[i]->bind(i+1);
    shader->setUniformValue("qt_Texture0", (i+1));

    const uint triangleOffset = i*6*sizeof(uint);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (void*)triangleOffset);

    textures[i]->release(i+1);
  }

  indexBuffer->release();
  vertexBuffer->release();
  shader->release();
}

void frameHandler::slotSkyboxRenderingToggled(bool newValue) 
{ 
  DEBUG_FRAMEHANDLER("frameHandler::slotSkyboxRenderingToggled %d", newValue);

  if (newValue)
    // The user enabled the skybox rendering. Load the current frame pixmap as texture.
    // The child item has to load the next texture if the current frame changes while skyBox rendering is on.
    skyBox.loadTextureFromCubeMap(currentFrame.toImage());

  renderSkybox = newValue; 
  emit signalHandlerChanged(true, false); 
}
