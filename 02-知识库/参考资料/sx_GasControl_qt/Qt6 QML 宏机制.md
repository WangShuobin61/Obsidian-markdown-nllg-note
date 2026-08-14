```cpp
#ifndef AUTHMANAGER_H
#define AUTHMANAGER_H

#include <QObject>
#include <QQmlEngine>
#include <QString>
#include <QVariantList>
#include <memory>

class AuthRepository;

class AuthManager : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY_MOVE(AuthManager)
    QML_ELEMENT        // 标识为QML元素，可以在QML中使用
    QML_SINGLETON      // 标识为QML单例，只有一个实例
    
    // pro波迪 暴露“状态（State）表现在是什么？” 可实时更新，有固定含义，值可能变换，页面持续绑定，有信号触发时自动更新
    Q_PROPERTY(bool isAuthenticated READ isAuthenticated NOTIFY authenticationChanged)
    Q_PROPERTY(QString currentUser READ currentUser NOTIFY authenticationChanged) 
    Q_PROPERTY(QString currentRole READ currentRole NOTIFY authenticationChanged) 
    
public:
    explicit AuthManager(QObject *parent = nullptr);
    ~AuthManager();
    
    bool isAuthenticated() const { return m_isAuthenticated; }
    QString currentUser() const { return m_currentUser; }
    QString currentRole() const { return m_currentRole; }
    
    // 因喔卡波儿里 暴露“动作（Action）表要做什么？”  一般需要参数，可能有返回值，主动触发，不参与绑定
    // QML 调用的登录方法
    Q_INVOKABLE bool verifyLogin(const QString& username, const QString& password); 
    
    // QML 调用的修改密码方法
    Q_INVOKABLE bool changePassword(const QString& oldPassword, const QString& newPassword);
    
    // QML 调用的注册用户方法
    Q_INVOKABLE bool registerUser(const QString& superAdminPassword, const QString& username,
                                  const QString& role, const QString& password);
    
    // QML 调用的自定义用户列表方法
    Q_INVOKABLE QVariantList customUsers();
    
    // QML 调用的删除自定义用户方法
    Q_INVOKABLE bool deleteCustomUser(int userId);

    // QML 调用的重置自定义用户密码方法
    Q_INVOKABLE void resetCustomUserPassword(int userId);
    
    // QML 调用的登出方法
    Q_INVOKABLE void logout();
    
    // QML 调用的默认用户密码重置方法
    Q_INVOKABLE bool resetDatabase();
    
signals:
    void authenticationChanged();
    void loginFailed(const QString& message);
    void passwordChangeResult(bool success, const QString& message);
    void registerResult(bool success, const QString& message);
    void customUserDeleteResult(bool success, const QString& message);
    void customUserPasswordResetResult(bool success, const QString& message);
    
private:
    std::unique_ptr<AuthRepository> m_repository;
    bool m_isAuthenticated;
    QString m_currentUser;
    QString m_currentRole;
};

#endif // AUTHMANAGER_H
```

