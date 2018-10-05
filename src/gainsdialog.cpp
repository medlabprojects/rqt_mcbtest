#include "gainsdialog.h"
#include "ui_gainsdialog.h"

#include <QLineEdit>
#include <QDoubleValidator>
#include <QString>

GainsDialog::GainsDialog(quint8 motor, double p, double i, double d, QWidget *parent) :
  QDialog(parent),
  ui(new Ui::GainsDialog),
  motor_(motor),
  p_(p),
  i_(i),
  d_(d)
{
  ui->setupUi(this);

  // use Validator to ensure only a bounded decimal number is entered
  QDoubleValidator *doubleVal = new QDoubleValidator( 0.0, 1.0, 8, this ); // between 0-1, max of 8 digits after decimal
  doubleVal->setNotation(QDoubleValidator::StandardNotation);
  doubleVal->setLocale(QLocale::C);
  ui->lineEdit_P->setValidator( doubleVal );
  ui->lineEdit_I->setValidator( doubleVal );
  ui->lineEdit_D->setValidator( doubleVal );

  // display current gains
  ui->label_motorNum->setText(QString::number(motor_));
  ui->lineEdit_P->setText(QString::number(p_,'f',8));
  ui->lineEdit_I->setText(QString::number(i_,'f',8));
  ui->lineEdit_D->setText(QString::number(d_,'f',8));

  // connect lineEdit boxes
  connect(ui->lineEdit_P, SIGNAL(textEdited(QString)), this, SLOT(newP(QString)));
  connect(ui->lineEdit_I, SIGNAL(textEdited(QString)), this, SLOT(newI(QString)));
  connect(ui->lineEdit_D, SIGNAL(textEdited(QString)), this, SLOT(newD(QString)));

  // connect OK button
  connect(ui->buttonBox, SIGNAL(accepted()), this, SLOT(saveGainsAndExit()));
}

GainsDialog::~GainsDialog()
{
  delete ui;
}

void GainsDialog::newP(QString strP)
{
  // check that number is valid before storing
  if(isNumberValid(ui->lineEdit_P)){
    if(strP.toDouble()){ // conversion should not fail after validation, but better to be safe :)
      p_ = strP.toDouble();
    }
  }
}

void GainsDialog::newI(QString strI)
{
  // check that number is valid before storing
  if(isNumberValid(ui->lineEdit_I)){
    if(strI.toDouble()){ // conversion should not fail after validation, but better to be safe :)
      i_ = strI.toDouble();
    }
  }
}

void GainsDialog::newD(QString strD)
{
  // check that number is valid before storing
  if(isNumberValid(ui->lineEdit_D)){
    if(strD.toDouble()){ // conversion should not fail after validation, but better to be safe :)
      d_ = strD.toDouble();
    }
  }
}

void GainsDialog::saveGainsAndExit()
{
  // emit signal with the new gains
  emit newGains(motor_, p_, i_, d_);

  // close this dialog window
  close();
}

bool GainsDialog::isNumberValid(QLineEdit* lineEdit)
{
  bool isValid = false;

  QString str = lineEdit->text();
  QDoubleValidator *val = (QDoubleValidator *) lineEdit->validator();
  int i=0; // not used, but needed by validate()
  QValidator::State st = val->validate(str, i);

  if (st == QValidator::Acceptable) {
    isValid = true;
  }

  return isValid;
}
