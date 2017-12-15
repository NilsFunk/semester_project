/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
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
 *   * Neither the name of the TU Darmstadt nor the names of its
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

#include <rqt_image_view/image_view.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>

namespace rqt_image_view 
{

ImageView::ImageView()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("ImageView");
}

void ImageView::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget(); // create QWidget
  ui_.setupUi(widget_); // extend the widget with all attributes and children from UI file

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_); // add widget to the user interface 

  subscriber_image_01_.shutdown();
  subscriber_image_02_.shutdown();

  // reset image on topic change
  //ui_.image_frame_2->setImage(QImage());

  QString topic_01 = "/hummingbird/vi_sensor/camera_depth/depth/expanded";
  QString topic_02 = "/hummingbird/vi_sensor/camera_depth/depth/clipped";
  QString topic_03 = "/hummingbird/vi_sensor/camera_depth/depth/horizon";
  QString topic_04 = "/hummingbird/vi_sensor/camera_depth/depth/disparity_top_view";
  QString topic_05 = "/hummingbird/vi_sensor/camera_depth/depth/expanded_top_view";

  QString transport = "raw";

  if (!topic_01.isEmpty())
  {
    image_transport::ImageTransport it(getNodeHandle());
    image_transport::TransportHints hints(transport.toStdString());
    try {
      subscriber_image_01_ = it.subscribe(topic_01.toStdString(), 1, &ImageView::callbackImage_1, this, hints);
      subscriber_image_02_ = it.subscribe(topic_02.toStdString(), 1, &ImageView::callbackImage_2, this, hints);
      subscriber_image_03_ = it.subscribe(topic_03.toStdString(), 1, &ImageView::callbackImage_3, this, hints);
      subscriber_image_04_ = it.subscribe(topic_04.toStdString(), 1, &ImageView::callbackImage_4, this, hints);
	  subscriber_image_05_ = it.subscribe(topic_05.toStdString(), 1, &ImageView::callbackImage_5, this, hints);

      //qDebug("ImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    }
  } 
  //ui_.image_frame->setOuterLayout(ui_.image_layout);

  QRegExp rx("([a-zA-Z/][a-zA-Z0-9_/]*)?"); //see http://www.ros.org/wiki/ROS/Concepts#Names.Valid_Names (but also accept an empty field)
}

void ImageView::shutdownPlugin()
{
  subscriber_image_01_.shutdown();
  subscriber_image_02_.shutdown();
}

void ImageView::callbackImage_1(const sensor_msgs::Image::ConstPtr& msg1)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::RGB8);
    conversion_mat_1_ = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg1);
      if (msg1->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_1_ = cv_ptr->image;
      } else if (msg1->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_1_, CV_GRAY2RGB);
      } else if (msg1->encoding == "16UC1" || msg1->encoding == "32FC1") {
        // scale / quantify
        double min = 0;
        double max = 10;
        if (msg1->encoding == "16UC1") max *= 1000;
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, conversion_mat_1_, CV_GRAY2RGB);
      } else {
        qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg1->encoding.c_str(), e.what());
		ui_.image_frame->setImage(QImage());
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg1->encoding.c_str(), e.what());
      ui_.image_frame->setImage(QImage());
    }
  }

  // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_1_.data, conversion_mat_1_.cols, conversion_mat_1_.rows, conversion_mat_1_.step[0], QImage::Format_RGB888); 
  ui_.image_frame->setImage(image);
}

void ImageView::callbackImage_2(const sensor_msgs::Image::ConstPtr& msg2)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr2 = cv_bridge::toCvShare(msg2, sensor_msgs::image_encodings::RGB8);
    conversion_mat_2_ = cv_ptr2->image;
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr2 = cv_bridge::toCvShare(msg2);
      if (msg2->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_2_ = cv_ptr2->image;
      } else if (msg2->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr2->image, conversion_mat_2_, CV_GRAY2RGB);
      } else if (msg2->encoding == "16UC1" || msg2->encoding == "32FC1") {
        // scale / quantify
        double min = 0;
        double max = 10;
        if (msg2->encoding == "16UC1") max *= 1000;
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr2->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, conversion_mat_2_, CV_GRAY2RGB);
      } else {
        qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg2->encoding.c_str(), e.what());
		ui_.image_frame_2->setImage(QImage());
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg2->encoding.c_str(), e.what());
      ui_.image_frame_2->setImage(QImage());
    }
  }

  // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_2_.data, conversion_mat_2_.cols, conversion_mat_2_.rows, conversion_mat_2_.step[0], QImage::Format_RGB888); 
  ui_.image_frame_2->setImage(image);
}

void ImageView::callbackImage_3(const sensor_msgs::Image::ConstPtr& msg3)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr3 = cv_bridge::toCvShare(msg3, sensor_msgs::image_encodings::RGB8);
    conversion_mat_3_ = cv_ptr3->image;
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr3 = cv_bridge::toCvShare(msg3);
      if (msg3->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_3_ = cv_ptr3->image;
      } else if (msg3->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr3->image, conversion_mat_3_, CV_GRAY2RGB);
      } else if (msg3->encoding == "16UC1" || msg3->encoding == "32FC1") {
        // scale / quantify
        double min = 0;
        double max = 10;
        if (msg3->encoding == "16UC1") max *= 1000;
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr3->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, conversion_mat_3_, CV_GRAY2RGB);
      } else {
        qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg3->encoding.c_str(), e.what());
		ui_.image_frame_3->setImage(QImage());
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg3->encoding.c_str(), e.what());
      ui_.image_frame_3->setImage(QImage());
    }
  }

  // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_3_.data, conversion_mat_3_.cols, conversion_mat_3_.rows, conversion_mat_3_.step[0], QImage::Format_RGB888); 
  ui_.image_frame_3->setImage(image);
}

void ImageView::callbackImage_4(const sensor_msgs::Image::ConstPtr& msg4)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr4 = cv_bridge::toCvShare(msg4, sensor_msgs::image_encodings::RGB8);
    conversion_mat_4_ = cv_ptr4->image;
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr4 = cv_bridge::toCvShare(msg4);
      if (msg4->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_4_ = cv_ptr4->image;
      } else if (msg4->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr4->image, conversion_mat_4_, CV_GRAY2RGB);
      } else if (msg4->encoding == "16UC1" || msg4->encoding == "32FC1") {
        // scale / quantify
        double min = 0;
        double max = 10;
        if (msg4->encoding == "16UC1") max *= 1000;
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr4->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, conversion_mat_4_, CV_GRAY2RGB);
      } else {
        qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg4->encoding.c_str(), e.what());
		ui_.image_frame_4->setImage(QImage());
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg4->encoding.c_str(), e.what());
      ui_.image_frame_4->setImage(QImage());
    }
  }

  // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_4_.data, conversion_mat_4_.cols, conversion_mat_4_.rows, conversion_mat_4_.step[0], QImage::Format_RGB888); 
  ui_.image_frame_4->setImage(image);
}

void ImageView::callbackImage_5(const sensor_msgs::Image::ConstPtr& msg5)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr5 = cv_bridge::toCvShare(msg5, sensor_msgs::image_encodings::RGB8);
    conversion_mat_5_ = cv_ptr5->image;
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr5 = cv_bridge::toCvShare(msg5);
      if (msg5->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_5_ = cv_ptr5->image;
      } else if (msg5->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr5->image, conversion_mat_5_, CV_GRAY2RGB);
      } else if (msg5->encoding == "16UC1" || msg5->encoding == "32FC1") {
        // scale / quantify
        double min = 0;
        double max = 10;
        if (msg5->encoding == "16UC1") max *= 1000;
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr5->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, conversion_mat_5_, CV_GRAY2RGB);
      } else {
        qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg5->encoding.c_str(), e.what());
		ui_.image_frame_5->setImage(QImage());
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg5->encoding.c_str(), e.what());
      ui_.image_frame_5->setImage(QImage());
    }
  }

  // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_5_.data, conversion_mat_5_.cols, conversion_mat_5_.rows, conversion_mat_5_.step[0], QImage::Format_RGB888); 
  ui_.image_frame_5->setImage(image);
}

}

PLUGINLIB_EXPORT_CLASS(rqt_image_view::ImageView, rqt_gui_cpp::Plugin)
