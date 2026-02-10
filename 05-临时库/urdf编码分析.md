### 1. 创建 src/viewmodels/defectmonitor.h (完整版)
```
#ifndef DEFECTMONITOR_H
#define DEFECTMONITOR_H

#include <QObject>
#include <QTimer>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QVector>
#include <QJsonObject>
#include <QJsonDocument>
#include <QtQml/qqmlregistration.h>
#include <cmath>

// KDL 库头文件
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

struct Point3D {
    double x;
    double y;
    double z;
};

class DefectMonitor : public QObject
{
    Q_OBJECT
    QML_ELEMENT

public:
    explicit DefectMonitor(QObject *parent = nullptr);
    ~DefectMonitor();

    Q_INVOKABLE void setWheelParameters(double radius, double tiltAngle = 15.0);
    Q_INVOKABLE bool loadRobotModel(const QString &urdfPath, const QString &
    baseLink, const QString &tipLink);

public slots:
    void startMonitoring(int intervalMs = 200);
    void stopMonitoring();

signals:
    // 给 2D 界面 (Area2)
    void newDefectDetected(double angle, double distancePercent, QString color);
    
    // 给 3D 界面 (Area8) - JSON 字符串格式 {"joint1": 1.57, ...}
    void jointsUpdated(QString jointsJson);

private slots:
    void checkDatabase();

private:
    Point3D calculateFK(const QVector<double>& joints);
    void processDefectData(const QVector<double>& joints, double wheelRotation);

    QTimer *m_timer;
    QString m_dbPath;
    int m_lastMaxId = 0;
    
    double m_wheelRadius = 300.0;
    double m_tiltAngle = 15.0;
    double m_baseOffsetX = 500.0;

    KDL::Tree m_tree;
    KDL::Chain m_chain;
    QSharedPointer<KDL::ChainFkSolverPos_recursive> m_fkSolver;
    bool m_modelLoaded = false;
};

#endif // DEFECTMONITOR_H
```
### 2. 创建 src/viewmodels/defectmonitor.cpp (完整版)
```
#include "defectmonitor.h"
#include <QDebug>
#include <QtMath>
#include <kdl_parser/kdl_parser.hpp>

DefectMonitor::DefectMonitor(QObject *parent) : QObject(parent)
{
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &DefectMonitor::checkDatabase);
    m_dbPath = "/home/wangshuobin/project/wheel_scanning/res/db/wheel_model_data.
    db";
}

DefectMonitor::~DefectMonitor()
{
    if (m_timer->isActive()) m_timer->stop();
}

void DefectMonitor::setWheelParameters(double radius, double tiltAngle)
{
    m_wheelRadius = radius;
    m_tiltAngle = tiltAngle;
}

bool DefectMonitor::loadRobotModel(const QString &urdfPath, const QString &
baseLink, const QString &tipLink)
{
    if (!kdl_parser::treeFromFile(urdfPath.toStdString(), m_tree)) {
        qCritical() << "Failed to construct KDL tree from URDF:" << urdfPath;
        return false;
    }
    if (!m_tree.getChain(baseLink.toStdString(), tipLink.toStdString(), m_chain)) {
        qCritical() << "Failed to get chain from" << baseLink << "to" << tipLink;
        return false;
    }
    m_fkSolver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
    m_modelLoaded = true;
    qDebug() << "Robot model loaded. Segments:" << m_chain.getNrOfSegments();
    return true;
}

void DefectMonitor::startMonitoring(int intervalMs)
{
    m_timer->start(intervalMs);
    qDebug() << "DefectMonitor started.";
}

void DefectMonitor::stopMonitoring()
{
    m_timer->stop();
}

void DefectMonitor::checkDatabase()
{
    const QString connectionName = "DefectMonitorConnection";
    {
        QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", connectionName);
        db.setDatabaseName(m_dbPath);
        if (!db.open()) return;

        QSqlQuery query(db);
        // 读取最新的一条数据用于实时展示
        // 注意：这里逻辑稍有修改，如果是为了实时动画，可能不需要 WHERE id > lastId，而是直接
        取最新
        // 但为了不漏掉缺陷点，我们还是遍历
        query.prepare("SELECT id, j1, j2, j3, j4, j5, j6, j7, wheel_angle FROM 
        defects WHERE id > :lastId ORDER BY id ASC");
        query.bindValue(":lastId", m_lastMaxId);
        
        if (query.exec()) {
            while (query.next()) {
                int id = query.value("id").toInt();
                QVector<double> joints(7);
                for(int i=0; i<7; ++i) joints[i] = query.value(QString("j%1").arg(i
                +1)).toDouble();
                double wheelAngle = query.value("wheel_angle").toDouble();

                processDefectData(joints, wheelAngle);

                if (id > m_lastMaxId) m_lastMaxId = id;
            }
        }
        db.close();
    }
    QSqlDatabase::removeDatabase(connectionName);
}

Point3D DefectMonitor::calculateFK(const QVector<double>& joints)
{
    if (!m_modelLoaded) return { 300.0, 0.0, 500.0 };

    unsigned int nj = m_chain.getNrOfJoints();
    KDL::JntArray q(nj);
    for (unsigned int i = 0; i < nj && i < (unsigned int)joints.size(); ++i) {
        q(i) = qDegreesToRadians(joints[i]); 
    }

    KDL::Frame p_out;
    if (m_fkSolver->JntToCart(q, p_out) < 0) return {0,0,0};

    return { p_out.p.x() * 1000.0, p_out.p.y() * 1000.0, p_out.p.z() * 1000.0 };
}

void DefectMonitor::processDefectData(const QVector<double>& joints, double 
wheelRotation)
{
    // 1. 发送给 3D 视图 (Area8)
    QJsonObject json;
    // 假设 URDF 中的关节名称为 joint1, joint2...
    for(int i=0; i<joints.size(); ++i) {
        // 注意：Web端通常需要弧度
        json[QString("joint%1").arg(i+1)] = qDegreesToRadians(joints[i]);
    }
    emit jointsUpdated(QJsonDocument(json).toJson(QJsonDocument::Compact));

    // 2. 发送给 2D 视图 (Area2)
    Point3D tip = calculateFK(joints);
    
    double dx = tip.x - m_baseOffsetX;
    double dy = tip.y; 
    
    double tiltRad = qDegreesToRadians(m_tiltAngle);
    double y_on_wheel = dy * qCos(tiltRad) + tip.z * qSin(tiltRad);
    double x_on_wheel = dx;
    
    double radius = qSqrt(x_on_wheel*x_on_wheel + y_on_wheel*y_on_wheel);
    double angleRad = qAtan2(y_on_wheel, x_on_wheel);
    double angleDeg = qRadiansToDegrees(angleRad);
    
    double finalAngle = angleDeg + wheelRotation;
    while (finalAngle < 0) finalAngle += 360.0;
    while (finalAngle >= 360.0) finalAngle -= 360.0;
    
    double distPercent = radius / m_wheelRadius;
    if (distPercent > 1.0) distPercent = 1.0; 
    
    emit newDefectDetected(finalAngle, distPercent, "red");
}
```
### 3. 修改 Area8.qml 以连接信号
在 Area8.qml 中，我们需要监听 DefectMonitor 的 jointsUpdated 信号，并调用 WebEngine 的 JS。

注意 ：由于 DefectMonitor 是在 Area2.qml 中实例化的（为了生命周期管理方便），我们最好把它提升为 全局单例 或者在 Main.qml 中实例化。但为了改动最小，我们可以利用 QML 的对象查找或信号转发。

更简单的做法 ：在 Area8.qml 里也实例化一个 DefectMonitor ？不，那样会有两个定时器读数据库。

最佳实践 ：在 Area2 中实例化的 defectMonitor 设置个 id ，但跨文件访问不方便。
建议：将 DefectMonitor 实例化放在 Main.qml 或 ModuleA.qml (父级)，然后通过属性传给 Area2 和 Area8 （ Area8 在 ModuleC ，跨度有点大）。

为了快速实现，我建议利用 QML 单例 (Singleton) 或者 C++ 上下文属性 。但既然我们已经用了 QML_ELEMENT ，我们可以把它变成一个全局对象。

这里我选择在 Area8.qml 中通过 Connections 连接到某个共享对象。
或者，简单粗暴点： 暂时让 Area8 自己也读一次数据库 （虽然不完美，但能跑）。
不，这太蠢了。

修正方案 ：
我们将 DefectMonitor 注册为 C++ 单例 。
修改 defectmonitor.h 中的 QML_ELEMENT 为 QML_SINGLETON 。这样 Area2 和 Area8 访问的是同一个实例。

请接受下方的文件修改，我会一次性搞定所有这些逻辑。

1. 修改 CMakeLists.txt (添加 KDL 依赖)
2. 创建 DefectMonitor C++ 类 (实现核心逻辑)
3. 修改 HTML (添加 JS 接口)
4. 修改 Area8.qml (接收信号并驱动 Web)

123456
456789
66666645645879879654456312