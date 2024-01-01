EAR Diagram

```mermaid
erDiagram
    USERS ||--o{ USER-ROLES : has
    ROLES ||--o{ USER-ROLES : has
    USERS {
        int UserID PK
        varchar Username
        varchar Password
        varchar Email
        date DateOfRegistration
    }
    ROLES {
        int RoleID PK
        varchar RoleName
    }
    USER-ROLES {
        int UserID FK
        int RoleID FK
    }

```