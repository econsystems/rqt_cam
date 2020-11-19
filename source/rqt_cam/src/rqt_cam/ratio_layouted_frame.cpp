/*
 * Copyright (c) 2020, e-consystems
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the e-consystems nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <rqt_cam/ratio_layouted_frame.h>

#include <assert.h>
#include <QMouseEvent>

namespace rqt_cam {
  // Constructor of class RatioLayoutedFrame
  RatioLayoutedFrame::RatioLayoutedFrame(QWidget* parent, Qt::WindowFlags flags)
                                          : QFrame()
                                          , outer_layout_(NULL)
                                          , aspect_ratio_(4, 3)
                                          , smoothImage_(false)
  {
    connect(this, SIGNAL(delayed_update()), this, SLOT(update()), Qt::QueuedConnection);
  }
  // Destructor of class RatioLayoutedFrame
  RatioLayoutedFrame::~RatioLayoutedFrame()
  {
  }
  /*****************************************************************************
  *  Name	:	getImage.
  *  Returns : const QImage& - image
  *  Description	:   This function is to get the image.
  *****************************************************************************/
  const QImage& RatioLayoutedFrame::getImage() const
  {
    return qimage_;
  }
  /*****************************************************************************
  *  Name	:	getImageCopy.
  *  Returns : QImage - image
  *  Description	:   This function is to get the copy of the image.
  *****************************************************************************/
  QImage RatioLayoutedFrame::getImageCopy() const
  {
    QImage img;
    qimage_mutex_.lock();
    img = qimage_.copy();
    qimage_mutex_.unlock();
    return img;
  }
  /*****************************************************************************
  *  Name	:	setImage.
  *  Returns : const QImage& image - image
  *  Description	:This function is to set the image in the correct aspect ratio.
  *****************************************************************************/
  void RatioLayoutedFrame::setImage(const QImage& image)//, QMutex* image_mutex)
  {
    qimage_mutex_.lock();
    qimage_ = image.copy();
    setAspectRatio(qimage_.width(), qimage_.height());
    qimage_mutex_.unlock();
    emit delayed_update();
  }
  /*****************************************************************************
  *  Name	:	resizeToFitAspectRatio.
  *  Description	:This function is to resie the layout to fit the aspect ratio.
  *****************************************************************************/
  void RatioLayoutedFrame::resizeToFitAspectRatio()
  {
    QRect rect = contentsRect();

    // reduce longer edge to aspect ration
    double width;
    double height;

    if (outer_layout_)
    {
      width = outer_layout_->contentsRect().width();
      height = outer_layout_->contentsRect().height();
    }
    else
    {
      // if outer layout isn't available, this will use the old
      // width and height, but this can shrink the display image if the
      // aspect ratio changes.
      width = rect.width();
      height = rect.height();
    }

    double layout_ar = width / height;
    const double image_ar = double(aspect_ratio_.width()) /
                            double(aspect_ratio_.height());
    if (layout_ar > image_ar)
    {
      // too large width
      width = height * image_ar;
    }
    else
    {
      // too large height
      height = width / image_ar;
    }
    rect.setWidth(int(width + 0.5));
    rect.setHeight(int(height + 0.5));

    // resize taking the border line into account
    int border = lineWidth();
    resize(rect.width() + 2 * border, rect.height() + 2 * border);
  }
  /*****************************************************************************
  *  Name	:	setOuterLayout.
  *  Parameter1 : QVBoxLayout* outer_layout - Vertical layout box.
  *  Description	:   This function is to set the outer layout .
  *****************************************************************************/
  void RatioLayoutedFrame::setOuterLayout(QVBoxLayout* outer_layout)
  {
    outer_layout_ = outer_layout;
  }
  /*****************************************************************************
  *  Name	:	setInnerFrameMinimumSize.
  *  Parameter1 : const QSize& size - size of the frame
  *  Description	:   This function is to set the minimum inner frame size .
  *****************************************************************************/
  void RatioLayoutedFrame::setInnerFrameMinimumSize(const QSize& size)
  {
    int border = lineWidth();
    QSize new_size = size;
    new_size += QSize(2 * border, 2 * border);
    setMinimumSize(new_size);
    emit delayed_update();
  }
  /*****************************************************************************
  *  Name	:	setInnerFrameMaximumSize.
  *  Parameter1 : const QSize& size - size of the frame
  *  Description	:   This function is to set the maximum inner frame size .
  *****************************************************************************/
  void RatioLayoutedFrame::setInnerFrameMaximumSize(const QSize& size)
  {
    int border = lineWidth();
    QSize new_size = size;
    new_size += QSize(2 * border, 2 * border);
    setMaximumSize(new_size);
    emit delayed_update();
  }
  /*****************************************************************************
  *  Name	:	setInnerFrameFixedSize.
  *  Parameter1 : const QSize& size - size of the frame
  *  Description	:   This function is to set the fixed inner frame size .
  *****************************************************************************/
  void RatioLayoutedFrame::setInnerFrameFixedSize(const QSize& size)
  {
    setInnerFrameMinimumSize(size);
    setInnerFrameMaximumSize(size);
  }
  /*****************************************************************************
  *  Name	:	setAspectRatio.
  *  Parameter1 : unsigned short width - width of the frame
  *  Parameter1 : unsigned short height - height of the frame
  *  Description	:   This function is to set the Aspect ratio .
  *****************************************************************************/
  void RatioLayoutedFrame::setAspectRatio(unsigned short width,unsigned short height)
  {
    int divisor = greatestCommonDivisor(width, height);
    if (divisor != 0) {
      aspect_ratio_.setWidth(width / divisor);
      aspect_ratio_.setHeight(height / divisor);
    }
  }
  /*****************************************************************************
  *  Name	:	setAspectRatio.
  *  Parameter1 : QPaintEvent* event - Paint event
  *  Description	:   This function is to Paint the frame in the UI.
  *****************************************************************************/
  void RatioLayoutedFrame::paintEvent(QPaintEvent* event)
  {
    QPainter painter(this);
    qimage_mutex_.lock();
    if (!qimage_.isNull())
    {
      resizeToFitAspectRatio();
      // TODO: check if full draw is really necessary
      //QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
      //painter.drawImage(paint_event->rect(), qimage_);
      if (!smoothImage_) {
        painter.drawImage(contentsRect(), qimage_);
      } else {
        if (contentsRect().width() == qimage_.width()) {
          painter.drawImage(contentsRect(), qimage_);
        } else {
          QImage image = qimage_.scaled(contentsRect().width(),
                                        contentsRect().height(),
                                        Qt::KeepAspectRatio,
                                        Qt::SmoothTransformation);
          painter.drawImage(contentsRect(), image);
        }
      }
    } else {
      // default image with gradient
      QLinearGradient gradient(0, 0, frameRect().width(), frameRect().height());
      gradient.setColorAt(0, Qt::white);
      gradient.setColorAt(1, Qt::black);
      painter.setBrush(gradient);
      painter.drawRect(0, 0, frameRect().width() + 1, frameRect().height() + 1);
    }
    qimage_mutex_.unlock();
  }
  /*****************************************************************************
  *  Name	:	greatestCommonDivisor.
  *  Parameter1 : int value1
  *  Parameter1 : int value2
  *  Returns : int - greatest Common Divisor of value1 and value2.
  *  Description :This function is to get the greatest Common Divisor.
  *****************************************************************************/
  int RatioLayoutedFrame::greatestCommonDivisor(int value1, int value2)
  {
    if (value2==0)
    {
      return value1;
    }
    return greatestCommonDivisor(value2, value1 % value2);
  }
  /*****************************************************************************
  *  Name	:	mousePressEvent.
  *  Parameter1 : QMouseEvent * mouseEvent - Mouse pressed event
  *  Description	:   This function is called when mouse is pressed.
  *****************************************************************************/
  void RatioLayoutedFrame::mousePressEvent(QMouseEvent * mouseEvent)
  {
    if(mouseEvent->button() == Qt::LeftButton)
    {
      emit mouseLeft(mouseEvent->x(), mouseEvent->y());
    }
    QFrame::mousePressEvent(mouseEvent);
  }
  /*****************************************************************************
  *  Name	:	onSmoothImageChanged.
  *  Parameter1 : bool checked - true if smoothImage_ button is pressed.
  *  Description	:   This function is called when smoothImage_ is pressed.
  *****************************************************************************/
  void RatioLayoutedFrame::onSmoothImageChanged(bool checked) {
    smoothImage_ = checked;
  }

}// End of namespace rqt_cam
