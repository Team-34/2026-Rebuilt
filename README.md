# 2026-Rebuilt

Rebuilt: FRC game for the 2025–2026 season

## Troops

| Subsystem  | Leader   | Sub(s) |
| -- | -- | -- |
| Autonomous | Siefert  | Royals |
| Climber    | Olmsted  | Mitchell |
| Drivetrain | Siefert  | — |
| Intake     | Morelli  | Gatlin |
| LEDs       | Olmsted  | Gray |
| Shooter    | Higgins  | Walsh |
| Turret     | Helton   | Gray & Faulk |
| Vision     | Helton   | Gray & Faulk |
| Spindexer  | Morelli  | — |

## Motors

| Subsystem      | Can IDs | Function | Motor(s)                                | Controller(s)        | Gear Ratio(s)                 |
| --             | --      | --       | --                                      | --                   | --                            |
| **Drivetrain** | 0 - 19  | steering | 4x Falcon                               | TalonFX (_built-in_) |                               |
|                |         | drive    | 4x Kraken                               | TalonFX (_built-in_) | 6.75:1 (_L2 mechanism:motor_) |
||
| **Shooter**    | 20 - 29 | shooting | 2x Kraken x60                           | TalonFX (_built-in_) |                               |
|                |         | aiming   | 1x Johnson PLG                          | Talon SRX            |                               |
||
| **Feeder**     | 30 - 39 |          | 1x Kraken x44                           | TalonFX (_built-in_) |                               |
| **Climber**    | 40 - 49 |          | 2x Kraken x60                           | TalonFX (_built-in_) |                               |
| **Turret**     | 50 - 54 |          | 1x CTR Minion                           | TalonFXS             | 99:18 (_mechanism:motor_)     |
| **Spindexer**  | 55 - 59 |          | 1x Kraken (TBD)                         | TalonFX (_built-in_) |                               |
| **Intake**     | 60 - 63 |          | 2x CTR Minion                           | TalonFXS             |                               |

## Planned Button-map

| Button                        |   Subsystem       |   Action                         |   Activation Type |
| --                            | --                | --                               |   --              |
| **A**                         |   Spindexer,      |   Feed Forward                   |   whileTrue       |
| **A**                         |   Intake          |   Intake In                      |   whileTrue       |                                        
| **B**                         |   Spindexer,      |   Feed Reverse                   |   whileTrue       |
| **B**                         |   Intake          |   Intake Out                     |   whileTrue       | 
| **X**                         |   Intake          |   Deploy                         |   onTrue          |                                         
| **Y**                         |   Climber         |   Toggle Climber                 |   onTrue          |                                          
| **POV Up**                    |   Unbound         |   N/A                            |   N/A             |                                      
| **POV Left**                  |   Climber         |   Extend Manual                  |   onTrue          |                                      
| **POV Down**                  |   Unbound         |   N/A                            |   N/A             |                                    
| **POV Right**                 |   Climber         |   Retract Manual                 |   onTrue          |                                    
| **Left Bumper**               |   Turret          |   Move Left                      |   whileTrue       |                                     
| **Right Bumper**              |   Turret          |   Move Right                     |   whileTrue       |                                      
| **Left Trigger**              |   Turret          |   Point to Fiducial              |   onTrue          |                                  
| **Right Trigger**             |   Shooter,        |   Cycle Speed                    |   onTrue          |
| **Right Trigger**             |   Spindexer       |   Feed Forward                   |   whileTrue       |    
| **Left Stick (Movement)**     |   Swerve          |   Robot Translation              |   Permanent       |     
| **Left Stick (Press)**        |   Swerve          |   Combo Shieldwall               |   Permanent       |     
| **Right Stick (Movement)**    |   Swerve          |   Robot Rotation                 |   Permanent       |     
| **Right Stick (Press)**       |   Swerve          |   Combo Shieldwall               |   Permanent       |
| **Start**                     |   Swerve          |   Resets field-centric heading   |   onTrue          |
| **Back**                      |   Swerve          |   Resets field-centric heading   |   onTrue          |
