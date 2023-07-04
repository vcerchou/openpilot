#pragma once

#include <QPushButton>
#include <QStackedLayout>
#include <QWidget>

#include "common/util.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"


const int btn_size = 192;
const int img_size = (btn_size / 4) * 3;


// ***** onroad widgets *****
class OnroadAlerts : public QWidget {
  Q_OBJECT

public:
  OnroadAlerts(QWidget *parent = 0) : QWidget(parent) {};
  void updateAlert(const Alert &a);

protected:
  void paintEvent(QPaintEvent*) override;
  QString translateAlertText(const QString &text);
  
private:
  QColor bg;
  Alert alert = {};
};

class ExperimentalButton : public QPushButton {
  Q_OBJECT

public:
  explicit ExperimentalButton(QWidget *parent = 0);
  void updateState(const UIState &s);

private:
  void paintEvent(QPaintEvent *event) override;

  Params params;
  QPixmap engage_img;
  QPixmap experimental_img;
};

// container window for the NVG UI
class AnnotatedCameraWidget : public CameraWidget {
  Q_OBJECT
  Q_PROPERTY(float speed MEMBER speed);
  Q_PROPERTY(QString speedUnit MEMBER speedUnit);
  Q_PROPERTY(float setSpeed MEMBER setSpeed);
  Q_PROPERTY(float speedLimit MEMBER speedLimit);
  Q_PROPERTY(bool is_cruise_set MEMBER is_cruise_set);
  Q_PROPERTY(bool has_eu_speed_limit MEMBER has_eu_speed_limit);
  Q_PROPERTY(bool has_us_speed_limit MEMBER has_us_speed_limit);
  Q_PROPERTY(bool is_metric MEMBER is_metric);

  Q_PROPERTY(bool dmActive MEMBER dmActive);
  Q_PROPERTY(bool hideDM MEMBER hideDM);
  Q_PROPERTY(bool rightHandDM MEMBER rightHandDM);
  Q_PROPERTY(int status MEMBER status);
  Q_PROPERTY(float enginerpm MEMBER enginerpm);
  Q_PROPERTY(float lead_d_rel MEMBER lead_d_rel);
  Q_PROPERTY(int lead_status MEMBER lead_status);
  Q_PROPERTY(bool buttonColorSpeed MEMBER buttonColorSpeed);
  Q_PROPERTY(bool left_blindspot MEMBER left_blindspot);
  Q_PROPERTY(bool right_blindspot MEMBER right_blindspot);
  Q_PROPERTY(float steerAngle MEMBER steerAngle);
  Q_PROPERTY(bool gps_state MEMBER gps_state);
  Q_PROPERTY(int gpsSatelliteCount MEMBER gpsSatelliteCount);
  Q_PROPERTY(float gpsBearing MEMBER gpsBearing);
  Q_PROPERTY(float gpsVerticalAccuracy MEMBER gpsVerticalAccuracy);
  Q_PROPERTY(float gpsAltitude MEMBER gpsAltitude);
  Q_PROPERTY(float gpsAccuracy MEMBER gpsAccuracy);

public:
  explicit AnnotatedCameraWidget(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);

private:
  void drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity);
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  void drawTextColor(QPainter &p, int x, int y, const QString &text, const QColor &color);

  ExperimentalButton *experimental_btn;
  QPixmap dm_img;
  QPixmap steer_img;
  QPixmap gps_img, direction_img;
  float speed;
  QString speedUnit;
  float setSpeed;
  float speedLimit;
  bool is_cruise_set = false;
  bool is_metric = false;
  bool dmActive = false;
  bool hideDM = false;
  bool rightHandDM = false;
  float dm_fade_state = 1.0;
  bool has_us_speed_limit = false;
  bool has_eu_speed_limit = false;
  bool v_ego_cluster_seen = false;
  int status = STATUS_DISENGAGED;
  std::unique_ptr<PubMaster> pm;
  float enginerpm; 
  int skip_frame_count = 0;
  bool wide_cam_requested = false;
  float lead_d_rel = 0;
  int lead_status;
  bool buttonColorSpeed = false;
  bool left_blindspot, right_blindspot = false;
  float steerAngle = 0;
  bool gps_state = false;
  int gpsSatelliteCount = 0;
  float gpsBearing, gpsVerticalAccuracy, gpsAltitude, gpsAccuracy = 0;

protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat() override;
  void drawLaneLines(QPainter &painter, const UIState *s);
  void drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd);
  void drawHud(QPainter &p);
  void drawDriverState(QPainter &painter, const UIState *s);
  void drawIconRotate(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity, float angle);
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }
  inline QColor orangeColor(int alpha = 255) { return QColor(255, 149, 0, alpha); }
  inline QColor pinkColor(int alpha = 255) { return QColor(255, 191, 191, alpha); }
  inline QColor limeColor(int alpha = 255) { return QColor(120, 255, 120, alpha); }

  double prev_draw_t = 0;
  FirstOrderFilter fps_filter;
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }

private:
  void paintEvent(QPaintEvent *event);
  void mousePressEvent(QMouseEvent* e) override;
  OnroadAlerts *alerts;
  AnnotatedCameraWidget *nvg;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QWidget *map = nullptr;
  QHBoxLayout* split;


private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);
};
