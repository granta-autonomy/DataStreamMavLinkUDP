/*Wensocket PlotJuggler Plugin license(Faircode, Davide Faconti)

Copyright(C) 2018 Philippe Gauthier - ISIR - UPMC
Copyright(C) 2020 Davide Faconti
Permission is hereby granted to any person obtaining a copy of this software and
associated documentation files(the "Software"), to deal in the Software without
restriction, including without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and / or sell copies("Use") of the Software, and to permit persons
to whom the Software is furnished to do so. The above copyright notice and this permission
notice shall be included in all copies or substantial portions of the Software. THE
SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include "udp_mavlink_server.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QSettings>
#include <QDialog>
#include <mutex>
#include <QWebSocket>
#include <QIntValidator>
#include <QMessageBox>
#include <chrono>
#include <QNetworkDatagram>
#include <QNetworkInterface>

#include "ui_udp_mavlink_server.h"

class UdpServerDialog : public QDialog
{
public:
  UdpServerDialog() : QDialog(nullptr), ui(new Ui::UDPServerDialog)
  {
    ui->setupUi(this);
    ui->lineEditPort->setValidator(new QIntValidator());
    setWindowTitle("MAVlink UDP Server");

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
  }
  ~UdpServerDialog()
  {
    delete ui;
  }
  Ui::UDPServerDialog* ui;
};

UDP_Server::UDP_Server() : _running(false)
{
}

UDP_Server::~UDP_Server()
{
  shutdown();
}

bool UDP_Server::start(QStringList*)
{
  if (_running)
  {
    return _running;
  }

  if (parserFactories() == nullptr || parserFactories()->empty())
  {
    QMessageBox::warning(nullptr, tr("MAVlink UDP Server"), tr("No available MessageParsers"),
                         QMessageBox::Ok);
    _running = false;
    return false;
  }

  bool ok = false;

  UdpServerDialog dialog;

  // load previous values
  QSettings settings;
  QString address_str = settings.value("MAVLINK_UDP_Server::address", "127.0.0.1").toString();
  int port = settings.value("MAVLINK_UDP_Server::port", 9870).toInt();

  dialog.ui->lineEditAddress->setText(address_str);
  dialog.ui->lineEditPort->setText(QString::number(port));

  int res = dialog.exec();
  if (res == QDialog::Rejected)
  {
    _running = false;
    return false;
  }

  address_str = dialog.ui->lineEditAddress->text();
  port = dialog.ui->lineEditPort->text().toUShort(&ok);

  // save back to service
  settings.setValue("MAVLINK_UDP_Server::address", address_str);
  settings.setValue("MAVLINK_UDP_Server::port", port);

  QHostAddress address(address_str);

  bool success = true;
  success &= !address.isNull();

  _udp_socket = new QUdpSocket();

  if (!address.isMulticast())
  {
    success &= _udp_socket->bind(address, port);
  }
  else
  {
    success &= _udp_socket->bind(
        address, port, QAbstractSocket::ShareAddress | QAbstractSocket::ReuseAddressHint);

    // Add multicast group membership to all interfaces which support multicast.
    for (const auto& interface : QNetworkInterface::allInterfaces())
    {
      QNetworkInterface::InterfaceFlags iflags = interface.flags();
      if (interface.isValid() && !iflags.testFlag(QNetworkInterface::IsLoopBack) &&
          iflags.testFlag(QNetworkInterface::CanMulticast) &&
          iflags.testFlag(QNetworkInterface::IsRunning))
      {
        success &= _udp_socket->joinMulticastGroup(address, interface);
      }
    }
  }

  _running = true;

  connect(_udp_socket, &QUdpSocket::readyRead, this, &UDP_Server::processMessage);

  if (success)
  {
    qDebug() << tr("UDP listening on (%1, %2)").arg(address_str).arg(port);
  }
  else
  {
    QMessageBox::warning(nullptr, tr("MAVlink UDP Server"),
                         tr("Couldn't bind to UDP (%1, %2)").arg(address_str).arg(port),
                         QMessageBox::Ok);
    shutdown();
  }

  return _running;
}

void UDP_Server::shutdown()
{
  if (_running && _udp_socket)
  {
    _udp_socket->deleteLater();
    _running = false;
  }
}

void UDP_Server::processMessage()
{
  double stamp;
  std::unordered_map<std::string, PJ::TimeseriesBase<double>>::iterator series;
  std::lock_guard<std::mutex> lock(mutex());

  while (_udp_socket->hasPendingDatagrams())
  {
    QNetworkDatagram datagram = _udp_socket->receiveDatagram();

    QByteArray m = datagram.data();
    for (int a=0; a<m.count(); a++) {
      mavlink_message_t msg;
      if (mavlink_parse_char(MAVLINK_COMM_0, m.at(a), &msg, &status))
      {
        switch (msg.msgid)
        {
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV:
          mavlink_local_position_ned_cov_t ned_cov;
          mavlink_msg_local_position_ned_cov_decode(&msg, &ned_cov);

          series = dataMap().numeric.find("local_position_ned_cov/x");
          if (series == dataMap().numeric.end()) {
            if (initial_time_ms == 0) initial_time_ms = ned_cov.time_usec / 1000;

            dataMap().addNumeric("local_position_ned_cov/x");
            dataMap().addNumeric("local_position_ned_cov/y");
            dataMap().addNumeric("local_position_ned_cov/z");
            dataMap().addNumeric("local_position_ned_cov/vx");
            dataMap().addNumeric("local_position_ned_cov/vy");
            dataMap().addNumeric("local_position_ned_cov/vz");
          }

          stamp = (double)(ned_cov.time_usec / 1000 - initial_time_ms)/1000.0;
          
          dataMap().numeric.find("local_position_ned_cov/x")->second.pushBack({ stamp, ned_cov.x });
          dataMap().numeric.find("local_position_ned_cov/y")->second.pushBack({ stamp, ned_cov.y });
          dataMap().numeric.find("local_position_ned_cov/z")->second.pushBack({ stamp, ned_cov.z });
          dataMap().numeric.find("local_position_ned_cov/vx")->second.pushBack({ stamp, ned_cov.vx });
          dataMap().numeric.find("local_position_ned_cov/vy")->second.pushBack({ stamp, ned_cov.vy });
          dataMap().numeric.find("local_position_ned_cov/vz")->second.pushBack({ stamp, ned_cov.vz });

          emit dataReceived();

        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
          mavlink_local_position_ned_t ned;
          mavlink_msg_local_position_ned_decode(&msg, &ned);

          series = dataMap().numeric.find("local_position_ned/x");
          if (series == dataMap().numeric.end()) {
            if (initial_time_ms == 0) initial_time_ms = ned.time_boot_ms;

            dataMap().addNumeric("local_position_ned/x");
            dataMap().addNumeric("local_position_ned/y");
            dataMap().addNumeric("local_position_ned/z");
            dataMap().addNumeric("local_position_ned/vx");
            dataMap().addNumeric("local_position_ned/vy");
            dataMap().addNumeric("local_position_ned/vz");
          }

          stamp = (double)(ned.time_boot_ms - initial_time_ms)/1000.0;
          
          dataMap().numeric.find("local_position_ned/x")->second.pushBack({ stamp, ned.x });
          dataMap().numeric.find("local_position_ned/y")->second.pushBack({ stamp, ned.y });
          dataMap().numeric.find("local_position_ned/z")->second.pushBack({ stamp, ned.z });
          dataMap().numeric.find("local_position_ned/vx")->second.pushBack({ stamp, ned.vx });
          dataMap().numeric.find("local_position_ned/vy")->second.pushBack({ stamp, ned.vy });
          dataMap().numeric.find("local_position_ned/vz")->second.pushBack({ stamp, ned.vz });

          emit dataReceived();
        }
      }
    }
  }
  return;
}
