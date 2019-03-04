#ifndef GAINSDIALOG_H
#define GAINSDIALOG_H

#include <QDialog>
#include "ui_gainsdialog.h"
#include <QLineEdit>
#include <QString>

class GainsDialog : public QDialog
{
  Q_OBJECT

public:
  explicit GainsDialog(quint8 motor, double p, double i, double d, QWidget *parent = 0);
  ~GainsDialog();

protected slots:
  void newP(QString strP);
  void newI(QString strI);
  void newD(QString strD);
  void saveGainsAndExit();

private:
  Ui::GainsDialog_ui *ui;

  bool isNumberValid(QLineEdit* lineEdit);
  quint8 motor_;
  double p_;
  double i_;
  double d_;

signals:
  void newGains(quint8 motor, double p, double i, double d);
};

#endif // GAINSDIALOG_H
