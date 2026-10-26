plugins {
    id("dev.frozenmilk.teamcode") version "12.0.0-1.2.3"
    id("dev.frozenmilk.sinister.sloth.load") version "0.3.2"
}

ftc {
    sdk.TeamCode("12.0.0")
    implementation(dairy.Sloth)
}

repositories {
    google()
    mavenCentral()
    maven("https://repo.dairy.foundation/releases")
}

dependencies {
    implementation("com.bylazar.sloth:fullpanels:0.3.2+1.0.13")
    implementation("com.pedropathing:revhub:3.0.2")
    implementation("com.pedropathing:tuning:1.0.2")
    implementation("org.solverslib:core:0.3.6")
    implementation("org.solverslib:pedroPathing:0.3.6")
}

android {
    sourceSets {
        getByName("main") {
            manifest.srcFile("TeamCode/src/main/AndroidManifest.xml")
            java.setSrcDirs(listOf("TeamCode/src/main/java"))
            res.setSrcDirs(listOf("TeamCode/src/main/res"))
        }
    }
}
