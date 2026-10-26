<div dir="rtl" lang="he">

# תבנית FTC ומדריך תכנות של קבוצה 25262

[English](README.md)

בנו את קוד הרובוט של הקבוצה באמצעות Java,‏ FTC SDK, פקודות SolversLib, מעקב מסלולים Pedro 3,‏ Panels ו־Sloth. המאגר הוא גם נקודת התחלה עובדת וגם רצף דוגמאות קטנות שאפשר להעתיק ולהתאים.

**Starter TeleOp הפעיל דורש ארבעה מנועי הנעה מסוג mecanum,‏ goBILDA Pinpoint וגלגלי אודומטריה תואמים.** הגדירו ומדדו את הרובוט לפני נהיגה. שיעורי מנוע וסרוו דורשים רק את ההתקנים הנקובים בהם; שיעורים 01 ו־07 אינם דורשים ציוד היקפי לרובוט.

## התחילו כאן

1. קראו [התקנה והרצה ראשונה](doc/guide/he/01-setup.md). הגדירו את Gradle JVM בסביבת הפיתוח ל־**Java 25**, התקינו Android SDK וסנכרנו את פרויקט השורש.
2. התאימו את תצורת Robot Controller ל[טבלת החומרה](#required-hardware). ערכו את [Constants.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java): שמות, כיוונים, סוג גלגלי אודומטריה והיסטים עם סימן.
3. המתינו לסיום סנכרון Gradle הראשון. סנכרנו שוב אחרי שינוי תלויות או הגדרות בנייה, או כשה־IDE מבקש; עריכות Java רגילות בדרך כלל אינן דורשות זאת. קוד היישום משתמש ב־Java 8 bytecode אף ש־Gradle משתמש ב־Java 25.
4. חברו ובחרו את הבקר ב־Android Studio/IntelliJ, בחרו את תצורת ההרצה **Android App** ולחצו **Run ▶** כדי [לבנות ולהתקין את היישום](doc/guide/he/01-setup.md#install-and-run). בדרך כלל אין צורך ב־Build נפרד. שמרו על תאימות Driver Station ל־FTC SDK 12.0.
5. אמתו כיוונים ואודומטריה ובחרו **Starter TeleOp** ב־Driver Station. התחילו בתנועה יחסית לרובוט במהירות נמוכה. אוטונומי דורש מדידה וכוונון נוספים.

לשיעור ראשון ללא מנועים, העתיקו והפעילו את [01 מחזור חיים](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example01Lifecycle.java). כל דוגמה לימודית מושבתת עד שתפעילו אותה במכוון.

<a id="required-hardware"></a>
## חומרה נדרשת

| שימוש | שם בתצורת הרובוט | התקן |
|---|---|---|
| תוכנית התחלה / דוגמאות נהיגה / כוונון | `frontLeft` | מנוע DC קדמי שמאלי ל־mecanum |
| כנ״ל | `frontRight` | מנוע DC קדמי ימני ל־mecanum |
| כנ״ל | `backLeft` | מנוע DC אחורי שמאלי ל־mecanum |
| כנ״ל | `backRight` | מנוע DC אחורי ימני ל־mecanum |
| כנ״ל | `odo` | התקן goBILDA Pinpoint בחיבור I2C, מחובר לגלגלי אודומטריה תואמים |
| שיעורי מנוע וקבוצות פקודות בלבד | `exampleMotor` | מנוע DC נפרד |
| שיעורי סרוו וקבוצות פקודות בלבד | `exampleServo` | סרוו מיקום |

נדרשים גם בקר או hub של FTC, סוללה, Driver Station ובקר משחק. כל השמות חייבים להתאים בדיוק, כולל אותיות גדולות וקטנות. אפשר לשנות את שמות הדוגמה בקוד כדי להתאים לתצורה שלכם. אין מעבר אוטומטי למקודדי גלגלי ההנעה כש־Pinpoint חסר.

היסטי גלגלי האודומטריה מתחילים כ־**ערכי אפס זמניים**, ומקדמי התנועה והבלימה הם **אומדנים שלא נמדדו**. קראו [נהיגה ואמידת מיקום](doc/guide/he/06-driving.md) ו[אוטונומי](doc/guide/he/07-autonomous.md) לפני הסתמכות על מיקום או מסלולים.

## מה הפרויקט מספק

| רכיב | תפקיד |
|---|---|
| FTC SDK 12.0 | OpModes, בקרי משחק, hardwareMap, מנועים, סרוו וחיבור Driver Station |
| תוסף Dairy `12.0.0-1.2.3` | בניית Android בשורש והגדרת SDK ושרשרת כלים תואמים |
| SolversLib `0.3.6` | תתי־מערכות, מתזמן פקודות, קישור כפתורים והרכבת פקודות |
| Pedro core/revhub `3.0.2` | שילוב חומרת mecanum, אמידת מיקום Pinpoint ומעקב מסלולים Foresight |
| Pedro tuning `1.0.2` | תשתית כוונון; התבנית כוללת גם כלי מדידה מושבת |
| PoseFactory | כתיבת תנוחות מגרש במעלות ותנוחות זמן ריצה ברדיאנים |
| Sloth `0.3.2` | התקנה מהירה אופציונלית של קוד קבוצה לאחר התקנת יישום מלאה |
| Sloth FullPanels `0.3.2+1.0.13` | טלמטריה והגדרות בדפדפן התואמות ל־Sloth |

אל תוסיפו FullPanels רגיל לצד גרסת Sloth. פתחו את פרויקט השורש ב־IDE:‏ TeamCode היא תיקיית מקורות ביישום Dairy זה, ולא תת־פרויקט Gradle נפרד.

<a id="learning-path"></a>
## מסלול הלמידה

| פרק | מה תלמדו |
|---|---|
| [01 התקנה](doc/guide/he/01-setup.md) | כלים, בנייה, התקנה בהתקן והרצה ראשונה |
| [02 מבנה הפרויקט](doc/guide/he/02-structure.md) | היכן הקוד שייך וכיצד Gradle אורז אותו |
| [03 יסודות Java ו־FTC](doc/guide/he/03-foundations.md) | מחלקות, למבדות, ספקים, מחזור חיים, קלט וטלמטריה |
| [04 פקודות](doc/guide/he/04-commands.md) | ברירות מחדל, בעלות, קבוצות, ביטול ומגבלות זמן |
| [05 חומרה](doc/guide/he/05-hardware.md) | העתקת תתי־מערכות מנוע וסרוו והוספת מנגנונים |
| [06 נהיגה](doc/guide/he/06-driving.md) | Mecanum,‏ Pinpoint, מערכות צירים, שמות, היסטים ובקרות |
| [07 אוטונומי](doc/guide/he/07-autonomous.md) | PoseFactory, מסלולים, אינטרפולציית זווית ושילוב פקודות |
| [08 Panels,‏ Sloth וכוונון](doc/guide/he/08-panels-tuning.md) | חיבור דפדפן, הגדרות חיות מול תמונות מצב והתקנה |
| [09 פתרון בעיות](doc/guide/he/09-troubleshooting.md) | אבחון בעיות נפוצות בהתקנה, בהעברת קוד וברובוט |
| [10 עבודת צוות](doc/guide/he/10-workflow.md) | העתקת דוגמאות, הוספת תכונות, סקירה ומדידת שינויים |

## דוגמאות להעתקה

| שיעור | מקור | דרישות |
|---|---|---|
| 01 | [מחזור חיים וטלמטריה](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example01Lifecycle.java) | ללא ציוד היקפי |
| 02 | [תת־מערכת מנוע](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example02Motor.java) | `exampleMotor` |
| 03 | [תת־מערכת סרוו](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example03Servo.java) | `exampleServo` |
| 04 | [קבוצות פקודות](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example04CommandGroups.java) | שני התקני הדוגמה |
| 05 | [Mecanum ו־Pinpoint](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example05Mecanum.java) | חומרת הנעה מלאה |
| 06 | [אוטונומי בקו ובעקומה](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example06Autonomous.java) | חומרת הנעה מלאה ועוקב מכוונן |
| 07 | [הגדרות Panels](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example07Panels.java) | ללא ציוד היקפי |

העתיקו שיעור לחבילת `opmodes`, שנו את הצהרת החבילה, את שם הקובץ והמחלקה ואת שם ה־OpMode המוצג, ואז הסירו `@Disabled`. הגדירו רק את ההתקנים שהשיעור דורש. פעלו לפי [הליך ההעתקה המלא](doc/guide/he/10-workflow.md#copy-an-example).

## בקרות תוכנית ההתחלה

כל הבקרות משתמשות ב־**gamepad 1**. הסטיק השמאלי מזיז, ציר X בסטיק הימני מסובב, ה־bumper השמאלי מפחית הספק מרבי מ־`0.4` ל־`0.2`,‏ X מחליף נהיגה ביחס למגרש ו־D-pad UP מגדיר את המקום והכיוון הנוכחיים כתנוחת אפס. איפוס תנוחה אינו מזיז את הרובוט. התחילו בנהיגה יחסית לרובוט; נהיגה יחסית למגרש תלויה בקביעת זווית נכונה. סטיקים ממורכזים בתוך התחום המת מבקשים אפס תנועה.

## היסטוריה ומקורות

קוד הרובוט הייחודי לעונה הקודמת, כולל הגדרותיו, נשמר ב[ארכיון שלפני התבנית](doc/archive/pre-template/INDEX.md). הוא **אינו מקומפל או נרשם כ־OpModes**. יש בו בעיות מוכרות הייחודיות לרובוט ההוא והוא אינו תוכנית ההתחלה הנוכחית.

[CHANGELOG.md](CHANGELOG.md) מתעד היסטוריית פיתוח. [סקירת השדרוג](doc/UPGRADE_REVIEW.md) היא תמונת מצב היסטורית ולא מדריך התצורה הנוכחי. עץ `FtcRobotController` מכיל דוגמאות ומקורות מקוריים; Dairy מספק את תלות יישום הבקר.

שמרו את [LICENSE](LICENSE) והודעות צד שלישי הרלוונטיות בשימוש חוזר בתבנית. תיעוד מקורי מועיל: [FTC](https://ftc-docs.firstinspires.org/),‏ [Pedro](https://pedropathing.com/),‏ [SolversLib](https://github.com/FTC-23511/SolversLib),‏ [Panels](https://panels.bylazar.com/) ו־[Sloth](https://github.com/Dairy-Foundation/Sloth).

</div>
