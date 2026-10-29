#import <Foundation/Foundation.h>
#import <CoreMotion/CoreMotion.h>
#import <CoreLocation/CoreLocation.h>
#import <UIKit/UIKit.h>
#import <objc/runtime.h>
#import <math.h>
#import <float.h>
#import <stdlib.h>

enum { S_ACCEL, S_GYRO, S_MAG, S_USERACCEL, S_GRAVITY, S_ROTRATE, S_MAGCAL, S_ATT, S_COUNT };
enum { E_PRESSURE, E_RELALT, E_COUNT };
enum { U_MASK, U_APP_STATE, U_SCENE_STATE, U_PROTECTED_DATA, U_SCREEN_BRIGHTNESS, U_LIFECYCLE_NOTIFICATIONS, U_COUNT };

static BOOL gEnabled = YES;
static BOOL gExtraEnabled[E_COUNT];
static BOOL gAppEnabled = YES;
static BOOL gUsageMaskOption[U_COUNT];
static __thread BOOL gSBReadActualLifecycleState = NO;
static NSString *gAppBundleID = nil;
static BOOL gSensorEnabled[S_COUNT];
static BOOL gSensorAdv[S_COUNT];
static double gBase[S_COUNT];
static double gAxis[S_COUNT][3];
static double mPressure = 1, mRelAlt = 1;
static BOOL gGPSAdjustmentsEnabled = NO;
static double gGPSSmoothing = 0.65;        // 0 = raw fixes; positive smooths, negative amplifies.
static double gGPSSpeedMultiplier = 1.0;   // Scales each filtered location step; negative reverses.
static double gBtnOpacity = 0.92, gBtnSize = 54;

static NSArray *sensorKeys(){ return @[@"accel",@"gyro",@"mag",@"userAccel",@"gravity",@"rotRate",@"magCal",@"attitude"]; }
static NSArray *sensorLabels(){ return @[@"Accelerometer",@"Gyroscope",@"Magnetometer",@"User Acceleration",@"Gravity",@"Rotation Rate",@"Calibrated Mag",@"Attitude"]; }
static NSArray *axisLetters(){ return @[@"X",@"Y",@"Z"]; }
static BOOL SBIsGPSControlKey(NSString *key){ return [key isEqualToString:@"gpsSmoothing"] || [key isEqualToString:@"gpsSpeed"]; }
static NSString *SBFormatSettingValue(NSString *key, double value){ return SBIsGPSControlKey(key)?[NSString stringWithFormat:@"%.6g",value]:[NSString stringWithFormat:@"%.2f",value]; }

static NSUserDefaults *prefsDB(){ return [NSUserDefaults standardUserDefaults]; }
static double readD(id source, NSString *k, double def){ id v=[source objectForKey:k]; return v?[v doubleValue]:def; }
static BOOL readB(id source, NSString *k, BOOL def){ id v=[source objectForKey:k]; return v?[v boolValue]:def; }
static inline double M(int s, int a){ return gSensorAdv[s] ? gAxis[s][a] : gBase[s]; }
static BOOL SBShouldShowOverlay(void){ return NSBundle.mainBundle.bundleIdentifier.length > 0; }
static BOOL SBUsageMaskIsActive(void){ return gEnabled && gAppEnabled && gUsageMaskOption[U_MASK]; }
static UIApplicationState SBActualApplicationState(void){
    BOOL previous=gSBReadActualLifecycleState; gSBReadActualLifecycleState=YES;
    UIApplicationState state=[UIApplication sharedApplication].applicationState;
    gSBReadActualLifecycleState=previous;
    return state;
}
static UISceneActivationState SBActualSceneActivationState(UIScene *scene){
    if(!scene) return UISceneActivationStateUnattached;
    BOOL previous=gSBReadActualLifecycleState; gSBReadActualLifecycleState=YES;
    UISceneActivationState state=scene.activationState;
    gSBReadActualLifecycleState=previous;
    return state;
}

// Best-effort per-process masking of public app/screen state queries.
%hook UIApplication
- (UIApplicationState)applicationState{
    UIApplicationState actual=%orig;
    if(!gSBReadActualLifecycleState && SBUsageMaskIsActive() && gUsageMaskOption[U_APP_STATE]) return UIApplicationStateActive;
    return actual;
}
- (BOOL)isProtectedDataAvailable{
    BOOL actual=%orig;
    if(SBUsageMaskIsActive() && gUsageMaskOption[U_PROTECTED_DATA]) return YES;
    return actual;
}
%end

%hook UIScene
- (UISceneActivationState)activationState{
    UISceneActivationState actual=%orig;
    if(!gSBReadActualLifecycleState && SBUsageMaskIsActive() && gUsageMaskOption[U_SCENE_STATE]) return UISceneActivationStateForegroundActive;
    return actual;
}
%end

%hook UIScreen
- (CGFloat)brightness{
    CGFloat actual=%orig;
    if(SBUsageMaskIsActive() && gUsageMaskOption[U_SCREEN_BRIGHTNESS]) return 1.0;
    return actual;
}
%end

// Keep compass headings consistent with the per-axis magnetometer multipliers.
static __thread BOOL gSBRawCompassAxes=NO;
static double SBSpoofCompassHeading(CLHeading *heading, double originalHeading){
    if(!heading || !gEnabled || !gAppEnabled || !gSensorEnabled[S_MAG] || originalHeading<0) return originalHeading;
    double rawX=0, rawY=0;
    gSBRawCompassAxes=YES;
    @try {
        rawX=heading.x;
        rawY=heading.y;
    } @finally {
        gSBRawCompassAxes=NO;
    }
    if(!isfinite(rawX) || !isfinite(rawY) || hypot(rawX,rawY)<1e-9) return originalHeading;
    double mx=M(S_MAG,0), my=M(S_MAG,1);
    double scaledX=rawX*mx, scaledY=rawY*my;
    if(!isfinite(scaledX) || !isfinite(scaledY)) return originalHeading;
    double delta=(atan2(scaledY,scaledX)-atan2(rawY,rawX))*180.0/M_PI;
    double result=fmod(originalHeading+delta,360.0);
    if(result<0) result+=360.0;
    return result;
}

// Stateful, per-process GPS filter. It smooths the movement between consecutive
// fixes, scales the resulting trajectory, and keeps speed/course consistent with
// the coordinates that the injected app reads.
static BOOL gGPSFilterInitialized = NO;
static CLLocationCoordinate2D gGPSLastRawCoordinate = {0,0};
static CLLocationCoordinate2D gGPSSpoofCoordinate = {0,0};
static NSTimeInterval gGPSLastTimestamp = 0;
static double gGPSLastAccuracy = 0;
static double gGPSFilteredEastVelocity = 0, gGPSFilteredNorthVelocity = 0;
static BOOL gGPSHasFilteredVelocity = NO;
static double gGPSPreviousCourse = 0;
static BOOL gGPSHasCourse = NO;
static NSUInteger gGPSFilterGeneration = 0;
static char gGPSAdjustmentAssociationKey;
static NSUInteger gGPSCoordinateGetterCalls = 0;
static NSUInteger gGPSAdjustedFixCount = 0;
static BOOL gGPSHasLiveFix = NO;
static CLLocationCoordinate2D gGPSLiveRawCoordinate = {0,0};
static CLLocationCoordinate2D gGPSLiveAdjustedCoordinate = {0,0};
static NSMutableDictionary *gSBOriginalCoordinateIMPs = nil;
static CLLocationCoordinate2D SBSubclassCoordinateGetter(id self, SEL command);
static void SBInstallCLLocationSubclassHooks(void);

static void SBResetGPSFilter(void){
    __atomic_store_n(&gGPSCoordinateGetterCalls, 0, __ATOMIC_RELAXED);
    @synchronized([CLLocation class]) {
        gGPSFilterInitialized = NO;
        gGPSHasFilteredVelocity = NO;
        gGPSHasCourse = NO;
        gGPSLastTimestamp = 0;
        gGPSAdjustedFixCount = 0;
        gGPSHasLiveFix = NO;
        gGPSLiveRawCoordinate = CLLocationCoordinate2DMake(0,0);
        gGPSLiveAdjustedCoordinate = CLLocationCoordinate2DMake(0,0);
        gGPSFilterGeneration++;
    }
}

static CLLocationCoordinate2D SBCoordinateByAddingMeters(CLLocationCoordinate2D origin, double east, double north){
    const double earthRadius = 6371000.0;
    if (!CLLocationCoordinate2DIsValid(origin) || !isfinite(east) || !isfinite(north)) return origin;
    double latRadians = origin.latitude * M_PI / 180.0;
    double cosine = cos(latRadians);
    if (!isfinite(cosine) || fabs(cosine) < 1e-6) return origin;
    double latitude = origin.latitude + (north / earthRadius) * 180.0 / M_PI;
    double longitude = origin.longitude + (east / (earthRadius * cosine)) * 180.0 / M_PI;
    if (!isfinite(latitude) || !isfinite(longitude)) return origin;
    longitude = fmod(longitude, 360.0);
    if (longitude > 180.0) longitude -= 360.0;
    if (longitude < -180.0) longitude += 360.0;
    CLLocationCoordinate2D result = CLLocationCoordinate2DMake(latitude, longitude);
    return CLLocationCoordinate2DIsValid(result) ? result : origin;
}

static void SBMetersBetweenCoordinates(CLLocationCoordinate2D from, CLLocationCoordinate2D to, double *east, double *north){
    const double earthRadius = 6371000.0;
    double deltaLongitude = to.longitude - from.longitude;
    if (deltaLongitude > 180.0) deltaLongitude -= 360.0;
    if (deltaLongitude < -180.0) deltaLongitude += 360.0;
    double meanLatitude = (from.latitude + to.latitude) * 0.5 * M_PI / 180.0;
    *east = deltaLongitude * M_PI / 180.0 * earthRadius * cos(meanLatitude);
    *north = (to.latitude - from.latitude) * M_PI / 180.0 * earthRadius;
}

// Positive values increase filter memory; negative values apply a bounded
// anti-smoothing gain, without clamping the stored/user-entered value.
static double SBAlphaForSmoothing(double smoothing, NSTimeInterval dt){
    if (!isfinite(smoothing) || !isfinite(dt) || dt <= 0.0) return 1.0;
    if (smoothing == 0.0) return 1.0;
    double magnitude = fabs(smoothing);
    double timeConstant = 0.05 + 4.0 * magnitude;
    double filteredAlpha = 1.0 - exp(-dt / timeConstant);
    if (smoothing < 0.0) {
        double antiStrength = 1.0 - exp(-magnitude);
        double alpha = 1.0 + antiStrength * (1.0 - filteredAlpha);
        return isfinite(alpha) ? alpha : 1.0;
    }
    return isfinite(filteredAlpha) ? filteredAlpha : 1.0;
}

static NSDictionary *SBGPSAdjustmentForLocation(CLLocation *location, CLLocationCoordinate2D rawCoordinate){
    if (!gEnabled || !gAppEnabled || !gGPSAdjustmentsEnabled || !CLLocationCoordinate2DIsValid(rawCoordinate)) return nil;
    @synchronized([CLLocation class]) {
        NSDictionary *cached = objc_getAssociatedObject(location, &gGPSAdjustmentAssociationKey);
        if ([cached isKindOfClass:[NSDictionary class]] && [cached[@"generation"] unsignedIntegerValue] == gGPSFilterGeneration) return cached;

        NSDate *date = location.timestamp;
        NSTimeInterval timestamp = date.timeIntervalSince1970;
        double accuracy = location.horizontalAccuracy;
        if (!isfinite(timestamp) || timestamp <= 0 || !isfinite(accuracy) || accuracy < 0) return nil;

        CLLocationCoordinate2D adjusted = rawCoordinate;
        BOOL hasSpeed = NO, hasCourse = NO;
        double adjustedSpeed = 0, adjustedCourse = 0;
        if (!gGPSFilterInitialized) {
            gGPSFilterInitialized = YES;
            gGPSLastRawCoordinate = rawCoordinate;
            gGPSSpoofCoordinate = rawCoordinate;
            gGPSLastTimestamp = timestamp;
            gGPSLastAccuracy = accuracy;
            gGPSFilteredEastVelocity = 0;
            gGPSFilteredNorthVelocity = 0;
            gGPSHasFilteredVelocity = NO;
            gGPSHasCourse = NO;
        } else {
            double dt = timestamp - gGPSLastTimestamp;
            if (dt <= 0) return nil; // Do not let stale/history fixes rewind the live filter.
            if (dt > 60.0) {
                // Rebase on the new raw fix without snapping the adjusted route back to it.
                gGPSLastRawCoordinate = rawCoordinate;
                gGPSLastTimestamp = timestamp;
                gGPSLastAccuracy = accuracy;
                gGPSFilteredEastVelocity = 0;
                gGPSFilteredNorthVelocity = 0;
                gGPSHasFilteredVelocity = NO;
                adjusted = gGPSSpoofCoordinate;
                hasSpeed = YES;
                adjustedSpeed = 0;
                hasCourse = gGPSHasCourse;
                adjustedCourse = gGPSPreviousCourse;
            } else {
                double east = 0, north = 0;
                SBMetersBetweenCoordinates(gGPSLastRawCoordinate, rawCoordinate, &east, &north);
                double rawDistance = hypot(east, north);
                double maxPlausibleJump = 40.0 + 70.0 * dt + 2.0 * (accuracy + gGPSLastAccuracy);
                if (!isfinite(rawDistance) || rawDistance > maxPlausibleJump) {
                    // Drop a single implausible GPS jump and rebase, rather than teleport.
                    gGPSLastRawCoordinate = rawCoordinate;
                    gGPSLastTimestamp = timestamp;
                    gGPSLastAccuracy = accuracy;
                    gGPSFilteredEastVelocity = 0;
                    gGPSFilteredNorthVelocity = 0;
                    gGPSHasFilteredVelocity = NO;
                    adjusted = gGPSSpoofCoordinate;
                    hasSpeed = YES;
                    adjustedSpeed = 0;
                    hasCourse = gGPSHasCourse;
                    adjustedCourse = gGPSPreviousCourse;
                } else {
                    double alpha = SBAlphaForSmoothing(gGPSSmoothing, dt);
                    double rawEastVelocity = east / dt;
                    double rawNorthVelocity = north / dt;
                    double previousEastVelocity = gGPSHasFilteredVelocity ? gGPSFilteredEastVelocity : 0.0;
                    double previousNorthVelocity = gGPSHasFilteredVelocity ? gGPSFilteredNorthVelocity : 0.0;
                    gGPSFilteredEastVelocity = alpha * rawEastVelocity + (1.0 - alpha) * previousEastVelocity;
                    gGPSFilteredNorthVelocity = alpha * rawNorthVelocity + (1.0 - alpha) * previousNorthVelocity;
                    gGPSHasFilteredVelocity = YES;

                    double speedScale = gGPSSpeedMultiplier;
                    double stepEast = gGPSFilteredEastVelocity * dt * speedScale;
                    double stepNorth = gGPSFilteredNorthVelocity * dt * speedScale;
                    if (!isfinite(stepEast) || !isfinite(stepNorth)) {
                        // Keep corrupt/overflowing arithmetic from producing invalid coordinates.
                        gGPSFilteredEastVelocity = 0;
                        gGPSFilteredNorthVelocity = 0;
                        gGPSHasFilteredVelocity = NO;
                        stepEast = 0;
                        stepNorth = 0;
                    }
                    adjusted = SBCoordinateByAddingMeters(gGPSSpoofCoordinate, stepEast, stepNorth);
                    double adjustedDistance = hypot(stepEast, stepNorth);
                    adjustedSpeed = isfinite(adjustedDistance / dt) ? adjustedDistance / dt : 0.0;
                    hasSpeed = YES;
                    if (adjustedDistance > 0.05 && isfinite(adjustedDistance)) {
                        adjustedCourse = atan2(stepEast, stepNorth) * 180.0 / M_PI;
                        if (adjustedCourse < 0) adjustedCourse += 360.0;
                        gGPSPreviousCourse = adjustedCourse;
                        gGPSHasCourse = YES;
                    } else {
                        adjustedCourse = gGPSPreviousCourse;
                    }
                    hasCourse = gGPSHasCourse;
                    gGPSSpoofCoordinate = adjusted;
                    gGPSLastRawCoordinate = rawCoordinate;
                    gGPSLastTimestamp = timestamp;
                    gGPSLastAccuracy = accuracy;
                }
            }
        }

        NSDictionary *result = @{
            @"latitude": @(adjusted.latitude),
            @"longitude": @(adjusted.longitude),
            @"hasSpeed": @(hasSpeed),
            @"speed": @(adjustedSpeed),
            @"hasCourse": @(hasCourse),
            @"course": @(adjustedCourse),
            @"generation": @(gGPSFilterGeneration)
        };
        gGPSLiveRawCoordinate = rawCoordinate;
        gGPSLiveAdjustedCoordinate = adjusted;
        gGPSAdjustedFixCount++;
        gGPSHasLiveFix = YES;
        objc_setAssociatedObject(location, &gGPSAdjustmentAssociationKey, result, OBJC_ASSOCIATION_RETAIN_NONATOMIC);
        return result;
    }
}

static NSValue *SBClassPointerKey(Class cls){ return [NSValue valueWithPointer:(__bridge const void *)cls]; }
static IMP SBOriginalCoordinateIMPForClass(Class cls){
    @synchronized([CLLocation class]) {
        for(Class current=cls; current && current!=[NSObject class]; current=class_getSuperclass(current)){
            NSValue *value=gSBOriginalCoordinateIMPs[SBClassPointerKey(current)];
            if(value){ IMP original=NULL; [value getValue:&original]; return original; }
        }
    }
    return NULL;
}
static CLLocationCoordinate2D SBSubclassCoordinateGetter(id self, SEL command){
    IMP original=SBOriginalCoordinateIMPForClass(object_getClass(self));
    CLLocationCoordinate2D raw;
    if(original) raw=((CLLocationCoordinate2D(*)(id,SEL))original)(self,command);
    else {
        IMP base=class_getMethodImplementation([CLLocation class],command);
        raw=((CLLocationCoordinate2D(*)(id,SEL))base)(self,command);
    }
    __atomic_add_fetch(&gGPSCoordinateGetterCalls,1,__ATOMIC_RELAXED);
    NSDictionary *adjusted=SBGPSAdjustmentForLocation((CLLocation*)self,raw);
    if(!adjusted) return raw;
    return CLLocationCoordinate2DMake([adjusted[@"latitude"] doubleValue],[adjusted[@"longitude"] doubleValue]);
}
static void SBInstallCLLocationSubclassHooks(void){
    Class base=objc_getClass("CLLocation");
    if(!base) return;
    int capacity=objc_getClassList(NULL,0);
    if(capacity<=0) return;
    Class __unsafe_unretained *classes=(Class __unsafe_unretained *)calloc((size_t)capacity,sizeof(Class));
    if(!classes) return;
    int count=objc_getClassList(classes,capacity);
    @synchronized([CLLocation class]) {
        if(!gSBOriginalCoordinateIMPs) gSBOriginalCoordinateIMPs=[NSMutableDictionary dictionary];
        for(int i=0;i<count;i++){
            Class cls=classes[i];
            if(!cls || cls==base) continue;
            Class parent=class_getSuperclass(cls);
            while(parent && parent!=base) parent=class_getSuperclass(parent);
            if(parent!=base) continue;
            unsigned methodCount=0;
            Method *methods=class_copyMethodList(cls,&methodCount);
            for(unsigned j=0;j<methodCount;j++){
                if(method_getName(methods[j])!=@selector(coordinate)) continue;
                IMP original=method_getImplementation(methods[j]);
                if(original==(IMP)SBSubclassCoordinateGetter) break;
                gSBOriginalCoordinateIMPs[SBClassPointerKey(cls)]=[NSValue valueWithBytes:&original objCType:@encode(IMP)];
                method_setImplementation(methods[j],(IMP)SBSubclassCoordinateGetter);
                break;
            }
            free(methods);
        }
    }
    free(classes);
}
static void SBRescanCLLocationSubclassesSoon(void){
    const NSTimeInterval delays[]={0.25,1.0,3.0};
    for(int i=0;i<3;i++) dispatch_after(dispatch_time(DISPATCH_TIME_NOW,(int64_t)(delays[i]*NSEC_PER_SEC)),dispatch_get_main_queue(),^{ SBInstallCLLocationSubclassHooks(); });
}

static void loadPrefs(void){
    NSUserDefaults *d=prefsDB();
    gAppBundleID=NSBundle.mainBundle.bundleIdentifier?:@"unknown";
    NSDictionary *allApps=[d dictionaryForKey:@"appPrefs"];
    NSDictionary *app=[allApps[gAppBundleID] isKindOfClass:[NSDictionary class]]?allApps[gAppBundleID]:nil;
    id local=app?:d;
    gEnabled=YES;
    gAppEnabled=readB(local,@"appEnabled",YES);
    gUsageMaskOption[U_MASK]=readB(local,@"usageMaskEnabled",NO);
    gUsageMaskOption[U_APP_STATE]=readB(local,@"usageAppState",YES);
    gUsageMaskOption[U_SCENE_STATE]=readB(local,@"usageSceneState",YES);
    gUsageMaskOption[U_PROTECTED_DATA]=readB(local,@"usageProtectedData",YES);
    gUsageMaskOption[U_SCREEN_BRIGHTNESS]=readB(local,@"usageScreenBrightness",NO);
    gUsageMaskOption[U_LIFECYCLE_NOTIFICATIONS]=readB(local,@"usageLifecycleNotifications",NO);
    NSArray *keys=sensorKeys(); NSArray *ax=axisLetters();
    for(int s=0;s<S_COUNT;s++){
        NSString *ek=[NSString stringWithFormat:@"%@Enabled",keys[s]];
        gSensorEnabled[s]=readB(local,ek,YES);
        NSString *ak=[NSString stringWithFormat:@"%@Adv",keys[s]];
        gSensorAdv[s]=readB(local,ak,NO);
        gBase[s]=readD(local,keys[s],1);
        for(int a=0;a<3;a++) gAxis[s][a]=readD(local,[keys[s] stringByAppendingString:ax[a]],1);
    }
    gExtraEnabled[E_PRESSURE]=readB(local,@"pressureEnabled",YES);
    gExtraEnabled[E_RELALT]=readB(local,@"relAltEnabled",YES);
    mPressure=readD(local,@"pressure",1); mRelAlt=readD(local,@"relAlt",1);
    gGPSAdjustmentsEnabled=readB(local,@"gpsEnabled",NO);
    double storedSmoothing=readD(local,@"gpsSmoothing",0.65);
    double storedSpeed=readD(local,@"gpsSpeed",1.0);
    gGPSSmoothing=isfinite(storedSmoothing)?storedSmoothing:0.65;
    gGPSSpeedMultiplier=isfinite(storedSpeed)?storedSpeed:1.0;
    gBtnOpacity=readD(local,@"btnOpacity",0.92); gBtnSize=readD(local,@"btnSize",54);
}

static void savePrefs(void){
    NSUserDefaults *d=prefsDB();
    if(!gAppBundleID.length) gAppBundleID=NSBundle.mainBundle.bundleIdentifier?:@"unknown";
    NSMutableDictionary *apps=[[d dictionaryForKey:@"appPrefs"] mutableCopy]?:[NSMutableDictionary dictionary];
    NSMutableDictionary *app=[NSMutableDictionary dictionary];
    NSArray *keys=sensorKeys(); NSArray *ax=axisLetters();
    app[@"appEnabled"]=@(gAppEnabled);
    app[@"usageMaskEnabled"]=@(gUsageMaskOption[U_MASK]);
    app[@"usageAppState"]=@(gUsageMaskOption[U_APP_STATE]);
    app[@"usageSceneState"]=@(gUsageMaskOption[U_SCENE_STATE]);
    app[@"usageProtectedData"]=@(gUsageMaskOption[U_PROTECTED_DATA]);
    app[@"usageScreenBrightness"]=@(gUsageMaskOption[U_SCREEN_BRIGHTNESS]);
    app[@"usageLifecycleNotifications"]=@(gUsageMaskOption[U_LIFECYCLE_NOTIFICATIONS]);
    for(int s=0;s<S_COUNT;s++){
        app[[NSString stringWithFormat:@"%@Enabled",keys[s]]]=@(gSensorEnabled[s]);
        app[[NSString stringWithFormat:@"%@Adv",keys[s]]]=@(gSensorAdv[s]);
        app[keys[s]]=@(gBase[s]);
        for(int a=0;a<3;a++) app[[keys[s] stringByAppendingString:ax[a]]]=@(gAxis[s][a]);
    }
    app[@"pressureEnabled"]=@(gExtraEnabled[E_PRESSURE]);
    app[@"relAltEnabled"]=@(gExtraEnabled[E_RELALT]);
    app[@"pressure"]=@(mPressure); app[@"relAlt"]=@(mRelAlt);
    app[@"gpsEnabled"]=@(gGPSAdjustmentsEnabled);
    app[@"gpsSmoothing"]=@(gGPSSmoothing);
    app[@"gpsSpeed"]=@(gGPSSpeedMultiplier);
    app[@"btnOpacity"]=@(gBtnOpacity); app[@"btnSize"]=@(gBtnSize);
    apps[gAppBundleID]=app;
    [d setObject:apps forKey:@"appPrefs"];
    [d synchronize];

    NSString *doc=NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES).firstObject;
    if(doc){
        NSMutableDictionary *export=[app mutableCopy];
        export[@"bundleIdentifier"]=gAppBundleID;
        NSString *p=[doc stringByAppendingPathComponent:@"SensorBender.json"];
        NSData *j=[NSJSONSerialization dataWithJSONObject:export options:NSJSONWritingPrettyPrinted error:nil]; [j writeToFile:p atomically:YES];
    }
}

// live capture for readout
static CMAcceleration lastAccel={0}, lastAccelSpoof={0};
static CMRotationRate lastGyro={0}, lastGyroSpoof={0};
static CMAcceleration lastUserAccel={0}, lastGrav={0};
static CMMagneticField lastMag={0}, lastMagSpoof={0};
static CMCalibratedMagneticField lastMagCal={0}, lastMagCalSpoof={0};
static double lastCompassHeading=0,lastCompassHeadingSpoof=0;
static double lastRoll=0,lastPitch=0,lastYaw=0;
static double lastRollS=0,lastPitchS=0,lastYawS=0;

%hook CMAccelerometerData
- (CMAcceleration)acceleration{ CMAcceleration a=%orig; lastAccel=a; if(gEnabled && gAppEnabled && gSensorEnabled[S_ACCEL]){ CMAcceleration s=a; s.x*=M(S_ACCEL,0); s.y*=M(S_ACCEL,1); s.z*=M(S_ACCEL,2); lastAccelSpoof=s; return s; } lastAccelSpoof=a; return a; }
%end
%hook CMGyroData
- (CMRotationRate)rotationRate{ CMRotationRate r=%orig; lastGyro=r; if(gEnabled && gAppEnabled && gSensorEnabled[S_GYRO]){ CMRotationRate s=r; s.x*=M(S_GYRO,0); s.y*=M(S_GYRO,1); s.z*=M(S_GYRO,2); lastGyroSpoof=s; return s; } lastGyroSpoof=r; return r; }
%end
%hook CMMagnetometerData
- (CMMagneticField)magneticField{
    CMMagneticField real=%orig;
    lastMag=real;
    CMMagneticField spoof=real;
    if(gEnabled && gAppEnabled && gSensorEnabled[S_MAG]){ spoof.x*=M(S_MAG,0); spoof.y*=M(S_MAG,1); spoof.z*=M(S_MAG,2); }
    lastMagSpoof=spoof;
    return spoof;
}
%end

%hook CLHeading
- (CLLocationDirection)x{ CLLocationDirection v=%orig; if(!gSBRawCompassAxes && gEnabled && gAppEnabled && gSensorEnabled[S_MAG]) v*=M(S_MAG,0); return v; }
- (CLLocationDirection)y{ CLLocationDirection v=%orig; if(!gSBRawCompassAxes && gEnabled && gAppEnabled && gSensorEnabled[S_MAG]) v*=M(S_MAG,1); return v; }
- (CLLocationDirection)z{ CLLocationDirection v=%orig; if(!gSBRawCompassAxes && gEnabled && gAppEnabled && gSensorEnabled[S_MAG]) v*=M(S_MAG,2); return v; }
- (CLLocationDirection)magneticHeading{
    CLLocationDirection real=%orig;
    CLLocationDirection spoof=SBSpoofCompassHeading(self,real);
    lastCompassHeading=real; lastCompassHeadingSpoof=spoof;
    return spoof;
}
- (CLLocationDirection)trueHeading{ CLLocationDirection real=%orig; return SBSpoofCompassHeading(self,real); }
%end

%hook CLLocation
- (CLLocationCoordinate2D)coordinate{
    CLLocationCoordinate2D raw=%orig;
    __atomic_add_fetch(&gGPSCoordinateGetterCalls,1,__ATOMIC_RELAXED);
    NSDictionary *adjusted=SBGPSAdjustmentForLocation(self,raw);
    if(!adjusted) return raw;
    return CLLocationCoordinate2DMake([adjusted[@"latitude"] doubleValue],[adjusted[@"longitude"] doubleValue]);
}
- (CLLocationSpeed)speed{
    CLLocationSpeed raw=%orig;
    if(!gEnabled || !gAppEnabled || !gGPSAdjustmentsEnabled) return raw;
    CLLocationCoordinate2D ignored=self.coordinate; (void)ignored;
    NSDictionary *adjusted=objc_getAssociatedObject(self,&gGPSAdjustmentAssociationKey);
    if(![adjusted isKindOfClass:[NSDictionary class]]) return raw;
    if([adjusted[@"hasSpeed"] boolValue]) return [adjusted[@"speed"] doubleValue];
    return raw;
}
- (CLLocationDirection)course{
    CLLocationDirection raw=%orig;
    if(!gEnabled || !gAppEnabled || !gGPSAdjustmentsEnabled) return raw;
    CLLocationCoordinate2D ignored=self.coordinate; (void)ignored;
    NSDictionary *adjusted=objc_getAssociatedObject(self,&gGPSAdjustmentAssociationKey);
    if([adjusted isKindOfClass:[NSDictionary class]] && [adjusted[@"hasCourse"] boolValue]) return [adjusted[@"course"] doubleValue];
    return raw;
}
%end

// Core Location may vend private CLLocation subclasses. Hook direct coordinate
// overrides too, and rescan before common location-request entry points.
%hook CLLocationManager
- (void)startUpdatingLocation{ SBInstallCLLocationSubclassHooks(); SBRescanCLLocationSubclassesSoon(); %orig; }
- (void)requestLocation{ SBInstallCLLocationSubclassHooks(); SBRescanCLLocationSubclassesSoon(); %orig; }
- (void)startMonitoringSignificantLocationChanges{ SBInstallCLLocationSubclassHooks(); SBRescanCLLocationSubclassesSoon(); %orig; }
%end

%hook CMDeviceMotion
- (CMAcceleration)userAcceleration{ CMAcceleration a=%orig; lastUserAccel=a; if(gEnabled && gAppEnabled && gSensorEnabled[S_USERACCEL]){ a.x*=M(S_USERACCEL,0); a.y*=M(S_USERACCEL,1); a.z*=M(S_USERACCEL,2); } return a; }
- (CMAcceleration)gravity{ CMAcceleration a=%orig; lastGrav=a; if(gEnabled && gAppEnabled && gSensorEnabled[S_GRAVITY]){ a.x*=M(S_GRAVITY,0); a.y*=M(S_GRAVITY,1); a.z*=M(S_GRAVITY,2); } return a; }
- (CMRotationRate)rotationRate{ CMRotationRate r=%orig; if(gEnabled && gAppEnabled && gSensorEnabled[S_ROTRATE]){ r.x*=M(S_ROTRATE,0); r.y*=M(S_ROTRATE,1); r.z*=M(S_ROTRATE,2); } return r; }
- (CMCalibratedMagneticField)magneticField{
    CMCalibratedMagneticField real=%orig;
    lastMagCal=real;
    CMCalibratedMagneticField spoof=real;
    if(gEnabled && gAppEnabled && gSensorEnabled[S_MAGCAL]){ spoof.field.x*=M(S_MAGCAL,0); spoof.field.y*=M(S_MAGCAL,1); spoof.field.z*=M(S_MAGCAL,2); }
    lastMagCalSpoof=spoof;
    return spoof;
}
%end
%hook CMAttitude
- (double)roll{ double v=%orig; lastRoll=v; double s=gEnabled && gAppEnabled && gSensorEnabled[S_ATT] ? v*M(S_ATT,0):v; lastRollS=s; return s; }
- (double)pitch{ double v=%orig; lastPitch=v; double s=gEnabled && gAppEnabled && gSensorEnabled[S_ATT] ? v*M(S_ATT,1):v; lastPitchS=s; return s; }
- (double)yaw{ double v=%orig; lastYaw=v; double s=gEnabled && gAppEnabled && gSensorEnabled[S_ATT] ? v*M(S_ATT,2):v; lastYawS=s; return s; }
%end
%hook CMAltitudeData
- (NSNumber *)relativeAltitude{ NSNumber *n=%orig; if(gEnabled && gAppEnabled && gExtraEnabled[E_RELALT] && n) return @(n.doubleValue*mRelAlt); return n; }
- (NSNumber *)pressure{ NSNumber *n=%orig; if(gEnabled && gAppEnabled && gExtraEnabled[E_PRESSURE] && n) return @(n.doubleValue*mPressure); return n; }
%end

// PassThrough view to fix touch bug
@interface SBPassThroughView : UIView @end
@implementation SBPassThroughView
- (BOOL)pointInside:(CGPoint)point withEvent:(UIEvent *)event{
    for(UIView *v in self.subviews){
        if(!v.hidden && v.alpha>0.01 && v.userInteractionEnabled && [v pointInside:[self convertPoint:point toView:v] withEvent:event]) return YES;
    }
    return NO;
}
- (UIView *)hitTest:(CGPoint)point withEvent:(UIEvent *)event{
    for(UIView *v in self.subviews){
        if(!v.hidden && v.userInteractionEnabled){
            CGPoint p=[self convertPoint:point toView:v];
            UIView *h=[v hitTest:p withEvent:event];
            if(h) return h;
        }
    }
    return nil;
}
@end

@interface SBSensorBenderOverlay : NSObject
+ (instancetype)shared; - (void)installIfNeeded; - (void)reconcileVisibility; - (void)togglePanel; - (void)createWindow; - (void)scheduleAutoOpenPanel;
@end

@implementation SBSensorBenderOverlay {
    UIWindow *_floatWindow; UIWindow *_panelWindow;
    SBPassThroughView *_rootView; SBPassThroughView *_panelRootView;
    UIButton *_floatBtn; UIView *_panelBG; UIView *_card; UIScrollView *_scroll; UIStackView *_stack;
    UILabel *_gpsLiveLabel;
    NSMutableDictionary *_sliders; NSMutableDictionary *_valueLabels; NSMutableDictionary *_liveLabels;
    NSMutableDictionary *_sensorHeaders; NSMutableDictionary *_absRows; NSMutableDictionary *_advContainers;
    UISwitch *_enabledSwitch; UISwitch *_gpsAdjustmentsSwitch;
    NSMutableDictionary *_usageSwitches;
    NSTimer *_liveTimer;
    BOOL _didAutoOpenPanel; BOOL _autoOpenScheduled;
    CGPoint _panStart;
}
+ (instancetype)shared{ static SBSensorBenderOverlay *s; static dispatch_once_t t; dispatch_once(&t,^{s=[SBSensorBenderOverlay new];}); return s; }
- (void)installIfNeeded{
    if(!SBShouldShowOverlay() || SBActualApplicationState()==UIApplicationStateBackground) return;
    if(_floatWindow){
        dispatch_async(dispatch_get_main_queue(),^{
            if(SBShouldShowOverlay() && SBActualApplicationState()!=UIApplicationStateBackground){
                self->_floatWindow.hidden=NO;
                [self scheduleAutoOpenPanel];
            }
        });
        return;
    }
    dispatch_async(dispatch_get_main_queue(),^{
        if(SBShouldShowOverlay() && SBActualApplicationState()!=UIApplicationStateBackground && !self->_floatWindow) [self createWindow];
        else if(SBShouldShowOverlay() && self->_floatWindow) [self scheduleAutoOpenPanel];
    });
}
- (void)reconcileVisibility{
    if(SBShouldShowOverlay() && SBActualApplicationState()!=UIApplicationStateBackground) [self installIfNeeded];
    else dispatch_async(dispatch_get_main_queue(),^{ self->_panelWindow.hidden=YES; self->_floatWindow.hidden=YES; });
}
- (void)scheduleAutoOpenPanel{
    if(_didAutoOpenPanel || _autoOpenScheduled) return;
    _autoOpenScheduled=YES;
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW,(int64_t)(0.45*NSEC_PER_SEC)),dispatch_get_main_queue(),^{
        self->_autoOpenScheduled=NO;
        if(self->_didAutoOpenPanel || !SBShouldShowOverlay() || SBActualApplicationState()!=UIApplicationStateActive || !self->_panelWindow) return;
        self->_didAutoOpenPanel=YES;
        if(self->_panelWindow.hidden) [self togglePanel];
    });
}
- (UIWindowScene*)findWindowScene{
    UIWindowScene *fallback=nil;
    for(UIScene *scene in [UIApplication sharedApplication].connectedScenes){
        if(![scene isKindOfClass:[UIWindowScene class]]) continue;
        if(SBActualSceneActivationState(scene)==UISceneActivationStateForegroundActive) return (UIWindowScene*)scene;
        if(SBActualSceneActivationState(scene)==UISceneActivationStateForegroundInactive && !fallback) fallback=(UIWindowScene*)scene;
    }
    return fallback;
}
- (void)createWindow{
    if(!SBShouldShowOverlay() || SBActualApplicationState()==UIApplicationStateBackground) return;
    UIWindowScene *scene=[self findWindowScene];
    CGRect floatFrame=CGRectMake(20,90,gBtnSize,gBtnSize);
    // Float window - tiny, only button area, never blocks app
    if(scene){
        _floatWindow=[[UIWindow alloc] initWithWindowScene:scene];
        _panelWindow=[[UIWindow alloc] initWithWindowScene:scene];
    } else {
        _floatWindow=[[UIWindow alloc] initWithFrame:floatFrame];
        _panelWindow=[[UIWindow alloc] initWithFrame:[UIScreen mainScreen].bounds];
    }
    _floatWindow.frame=floatFrame; _floatWindow.windowLevel=UIWindowLevelAlert+100;
    _floatWindow.backgroundColor=[UIColor clearColor]; _floatWindow.hidden=NO; _floatWindow.userInteractionEnabled=YES;
    UIViewController *vc1=[UIViewController new]; vc1.view.backgroundColor=[UIColor clearColor];
    _rootView=[SBPassThroughView new]; _rootView.frame=CGRectMake(0,0,floatFrame.size.width,floatFrame.size.height); _rootView.backgroundColor=[UIColor clearColor];
    vc1.view=_rootView;
    _floatWindow.rootViewController=vc1; [_floatWindow makeKeyAndVisible];

    _floatBtn=[UIButton buttonWithType:UIButtonTypeCustom];
    _floatBtn.frame=CGRectMake(0,0,gBtnSize,gBtnSize); _floatBtn.layer.cornerRadius=gBtnSize/2;
    _floatBtn.backgroundColor=[UIColor systemBlueColor]; _floatBtn.alpha=gBtnOpacity;
    _floatBtn.layer.shadowColor=[UIColor blackColor].CGColor; _floatBtn.layer.shadowOpacity=0.12; _floatBtn.layer.shadowRadius=3; _floatBtn.layer.shadowOffset=CGSizeMake(0,1);
    [_floatBtn setTitle:@"SB" forState:UIControlStateNormal]; [_floatBtn setTitleColor:[UIColor whiteColor] forState:UIControlStateNormal]; _floatBtn.titleLabel.font=[UIFont boldSystemFontOfSize:15];
    _floatBtn.accessibilityLabel=@"SensorBender controls";
    [_floatBtn addTarget:self action:@selector(togglePanel) forControlEvents:UIControlEventTouchUpInside];
    UIPanGestureRecognizer *pan=[[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(handlePan:)];
    [_floatBtn addGestureRecognizer:pan];
    [_rootView addSubview:_floatBtn];

    // Panel window - full screen but hidden until opened
    _panelWindow.windowLevel=UIWindowLevelAlert+101; _panelWindow.backgroundColor=[UIColor clearColor]; _panelWindow.hidden=YES;
    UIViewController *vc2=[UIViewController new]; vc2.view.backgroundColor=[UIColor clearColor];
    _panelRootView=[SBPassThroughView new]; _panelRootView.frame=[UIScreen mainScreen].bounds; _panelRootView.backgroundColor=[UIColor clearColor];
    vc2.view=_panelRootView;
    _panelWindow.rootViewController=vc2;

    _panelBG=[[UIView alloc] initWithFrame:_panelRootView.bounds];
    _panelBG.autoresizingMask=UIViewAutoresizingFlexibleWidth|UIViewAutoresizingFlexibleHeight;
    _panelBG.backgroundColor=[UIColor colorWithWhite:0 alpha:0.38]; _panelBG.hidden=NO;
    UITapGestureRecognizer *tapBG=[[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(togglePanel)];
    [_panelBG addGestureRecognizer:tapBG];
    [_panelRootView addSubview:_panelBG];

    _card=[[UIView alloc] init]; _card.backgroundColor=[UIColor systemGroupedBackgroundColor]; _card.layer.cornerRadius=15; _card.layer.borderWidth=0.5; _card.layer.borderColor=[UIColor separatorColor].CGColor; _card.layer.masksToBounds=YES; _card.translatesAutoresizingMaskIntoConstraints=NO;
    [_panelBG addSubview:_card];
    [NSLayoutConstraint activateConstraints:@[
        [_card.centerXAnchor constraintEqualToAnchor:_panelBG.centerXAnchor],
        [_card.centerYAnchor constraintEqualToAnchor:_panelBG.centerYAnchor],
        [_card.widthAnchor constraintEqualToAnchor:_panelBG.widthAnchor multiplier:0.94],
        [_card.heightAnchor constraintEqualToAnchor:_panelBG.heightAnchor multiplier:0.88]
    ]];
    _scroll=[[UIScrollView alloc] init]; _scroll.translatesAutoresizingMaskIntoConstraints=NO; _scroll.showsVerticalScrollIndicator=YES; _scroll.alwaysBounceVertical=YES;
    [_card addSubview:_scroll];
    [NSLayoutConstraint activateConstraints:@[
        [_scroll.topAnchor constraintEqualToAnchor:_card.topAnchor constant:12],
        [_scroll.leadingAnchor constraintEqualToAnchor:_card.leadingAnchor constant:12],
        [_scroll.trailingAnchor constraintEqualToAnchor:_card.trailingAnchor constant:-12],
        [_scroll.bottomAnchor constraintEqualToAnchor:_card.bottomAnchor constant:-12]
    ]];
    _stack=[[UIStackView alloc] init]; _stack.axis=UILayoutConstraintAxisVertical; _stack.spacing=12; _stack.translatesAutoresizingMaskIntoConstraints=NO;
    [_scroll addSubview:_stack];
    [NSLayoutConstraint activateConstraints:@[
        [_stack.topAnchor constraintEqualToAnchor:_scroll.contentLayoutGuide.topAnchor],
        [_stack.leadingAnchor constraintEqualToAnchor:_scroll.contentLayoutGuide.leadingAnchor],
        [_stack.trailingAnchor constraintEqualToAnchor:_scroll.contentLayoutGuide.trailingAnchor],
        [_stack.bottomAnchor constraintEqualToAnchor:_scroll.contentLayoutGuide.bottomAnchor],
        [_stack.widthAnchor constraintEqualToAnchor:_scroll.frameLayoutGuide.widthAnchor]
    ]];
    _sliders=[NSMutableDictionary dictionary]; _valueLabels=[NSMutableDictionary dictionary]; _liveLabels=[NSMutableDictionary dictionary];
    _sensorHeaders=[NSMutableDictionary dictionary]; _absRows=[NSMutableDictionary dictionary]; _advContainers=[NSMutableDictionary dictionary];
    _usageSwitches=[NSMutableDictionary dictionary];
    [self buildCard];
    _liveTimer=[NSTimer scheduledTimerWithTimeInterval:0.25 target:self selector:@selector(updateLive) userInfo:nil repeats:YES];
    [self scheduleAutoOpenPanel];
}
- (void)handlePan:(UIPanGestureRecognizer*)p{
    CGPoint t=[p translationInView:_floatWindow];
    if(p.state==UIGestureRecognizerStateBegan) _panStart=_floatWindow.center;
    _floatWindow.center=CGPointMake(_panStart.x+t.x,_panStart.y+t.y);
    if(p.state==UIGestureRecognizerStateEnded){
        CGFloat mid=CGRectGetMidX([UIScreen mainScreen].bounds); CGFloat tx= _floatWindow.center.x < mid ? 36 : [UIScreen mainScreen].bounds.size.width-36;
        [UIView animateWithDuration:0.25 animations:^{ self->_floatWindow.center=CGPointMake(tx, self->_floatWindow.center.y); }];
    }
}
- (void)togglePanel{ BOOL show=_panelWindow.hidden; if(show){ [self refreshUI]; _panelWindow.hidden=NO; [_panelWindow makeKeyAndVisible]; } else { _panelWindow.hidden=YES; [_floatWindow makeKeyAndVisible]; } _floatBtn.alpha=show?MIN(gBtnOpacity,0.35):gBtnOpacity; }
- (UILabel*)section:(NSString*)t{ UILabel *l=[UILabel new]; l.text=t; l.font=[UIFont systemFontOfSize:13 weight:UIFontWeightSemibold]; l.textColor=[UIColor secondaryLabelColor]; l.numberOfLines=0; return l; }
- (UIView*)rowForKey:(NSString*)key label:(NSString*)lab min:(float)mn max:(float)mx{
    UIView *row=[UIView new];
    UILabel *ttl=[UILabel new]; ttl.text=lab; ttl.font=[UIFont systemFontOfSize:13 weight:UIFontWeightMedium]; ttl.translatesAutoresizingMaskIntoConstraints=NO; [ttl.widthAnchor constraintEqualToConstant:118].active=YES;
    UISlider *sl=[UISlider new]; sl.minimumValue=mn; sl.maximumValue=mx; double initialValue=[self cur:key]; sl.value=isfinite(initialValue)&&fabs(initialValue)<=FLT_MAX?(float)initialValue:(initialValue<0?mn:mx); sl.accessibilityIdentifier=key; [sl addTarget:self action:@selector(changed:) forControlEvents:UIControlEventValueChanged];
    UILabel *val=[UILabel new]; val.font=[UIFont monospacedDigitSystemFontOfSize:12 weight:UIFontWeightMedium]; val.textColor=[UIColor systemBlueColor]; val.text=SBFormatSettingValue(key,initialValue); val.translatesAutoresizingMaskIntoConstraints=NO; [val.widthAnchor constraintEqualToConstant:SBIsGPSControlKey(key)?88:52].active=YES; val.textAlignment=NSTextAlignmentRight; val.userInteractionEnabled=YES; UITapGestureRecognizer *tap=[[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(tapValue:)]; [val addGestureRecognizer:tap]; val.accessibilityIdentifier=key;
    _sliders[key]=sl; _valueLabels[key]=val;
    UIStackView *h=[[UIStackView alloc] initWithArrangedSubviews:@[ttl,sl,val]]; h.axis=UILayoutConstraintAxisHorizontal; h.spacing=8; h.alignment=UIStackViewAlignmentCenter; h.translatesAutoresizingMaskIntoConstraints=NO;
    [row addSubview:h]; [NSLayoutConstraint activateConstraints:@[[h.topAnchor constraintEqualToAnchor:row.topAnchor],[h.leadingAnchor constraintEqualToAnchor:row.leadingAnchor],[h.trailingAnchor constraintEqualToAnchor:row.trailingAnchor],[h.bottomAnchor constraintEqualToAnchor:row.bottomAnchor]]];
    return row;
}
- (UIView*)axisRowForSensor:(int)idx axis:(int)aIdx name:(NSString*)sensorName{
    NSArray *keys=sensorKeys(); NSString *base=keys[idx]; NSArray *ax=axisLetters(); NSString *letter=ax[aIdx];
    NSString *key=[base stringByAppendingString:letter];
    UIView *row=[UIView new]; UILabel *ttl=[UILabel new]; ttl.translatesAutoresizingMaskIntoConstraints=NO; [ttl.widthAnchor constraintEqualToConstant:118].active=YES;
    NSString *dispName=[sensorName isEqualToString:@"Attitude"] ? (@[@"Roll",@"Pitch",@"Yaw"][aIdx]) : letter;
    ttl.text=dispName; ttl.font=[UIFont systemFontOfSize:13 weight:UIFontWeightMedium]; ttl.textColor=[UIColor secondaryLabelColor];
    UISlider *sl=[UISlider new]; sl.minimumValue=-5; sl.maximumValue=5; sl.value=(float)gAxis[idx][aIdx]; sl.accessibilityIdentifier=key; sl.minimumTrackTintColor=[UIColor systemBlueColor]; [sl addTarget:self action:@selector(axisChanged:) forControlEvents:UIControlEventValueChanged];
    UILabel *val=[UILabel new]; val.font=[UIFont monospacedDigitSystemFontOfSize:12 weight:UIFontWeightMedium]; val.textColor=[UIColor systemBlueColor]; val.text=[NSString stringWithFormat:@"%.2f",sl.value]; val.translatesAutoresizingMaskIntoConstraints=NO; [val.widthAnchor constraintEqualToConstant:52].active=YES; val.textAlignment=NSTextAlignmentRight; val.userInteractionEnabled=YES; UITapGestureRecognizer *tap=[[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(tapValue:)]; [val addGestureRecognizer:tap]; val.accessibilityIdentifier=key;
    _sliders[key]=sl; _valueLabels[key]=val;
    // live label for axis
    UILabel *live=[UILabel new]; live.font=[UIFont systemFontOfSize:9]; live.textColor=[UIColor tertiaryLabelColor]; live.tag=1000+aIdx;
    UIStackView *h=[[UIStackView alloc] initWithArrangedSubviews:@[ttl,sl,val]]; h.axis=UILayoutConstraintAxisHorizontal; h.spacing=8; h.alignment=UIStackViewAlignmentCenter;
    [row addSubview:h]; h.translatesAutoresizingMaskIntoConstraints=NO;
    [NSLayoutConstraint activateConstraints:@[[h.topAnchor constraintEqualToAnchor:row.topAnchor],[h.leadingAnchor constraintEqualToAnchor:row.leadingAnchor],[h.trailingAnchor constraintEqualToAnchor:row.trailingAnchor],[h.bottomAnchor constraintEqualToAnchor:row.bottomAnchor]]];
    return row;
}
- (double)cur:(NSString*)k{
    if([k isEqualToString:@"accel"]) return gBase[S_ACCEL]; if([k isEqualToString:@"gyro"]) return gBase[S_GYRO]; if([k isEqualToString:@"mag"]) return gBase[S_MAG];
    if([k isEqualToString:@"userAccel"]) return gBase[S_USERACCEL]; if([k isEqualToString:@"gravity"]) return gBase[S_GRAVITY]; if([k isEqualToString:@"rotRate"]) return gBase[S_ROTRATE];
    if([k isEqualToString:@"magCal"]) return gBase[S_MAGCAL]; if([k isEqualToString:@"attitude"]) return gBase[S_ATT];
    if([k isEqualToString:@"pressure"]) return mPressure; if([k isEqualToString:@"relAlt"]) return mRelAlt;
    if([k isEqualToString:@"gpsSmoothing"]) return gGPSSmoothing; if([k isEqualToString:@"gpsSpeed"]) return gGPSSpeedMultiplier;
    NSArray *keys=sensorKeys(); NSArray *ax=axisLetters(); for(int s=0;s<S_COUNT;s++) for(int a=0;a<3;a++) if([[keys[s] stringByAppendingString:ax[a]] isEqualToString:k]) return gAxis[s][a];
    return 1;
}
- (void)setCur:(double)v key:(NSString*)k{
    if([k isEqualToString:@"accel"]) gBase[S_ACCEL]=v; else if([k isEqualToString:@"gyro"]) gBase[S_GYRO]=v; else if([k isEqualToString:@"mag"]) gBase[S_MAG]=v;
    else if([k isEqualToString:@"userAccel"]) gBase[S_USERACCEL]=v; else if([k isEqualToString:@"gravity"]) gBase[S_GRAVITY]=v; else if([k isEqualToString:@"rotRate"]) gBase[S_ROTRATE]=v;
    else if([k isEqualToString:@"magCal"]) gBase[S_MAGCAL]=v; else if([k isEqualToString:@"attitude"]) gBase[S_ATT]=v;
    else if([k isEqualToString:@"pressure"]) mPressure=v; else if([k isEqualToString:@"relAlt"]) mRelAlt=v;
    else if([k isEqualToString:@"gpsSmoothing"]) gGPSSmoothing=v;
    else if([k isEqualToString:@"gpsSpeed"]) gGPSSpeedMultiplier=v;
}
- (void)expandSliderForKey:(NSString*)key value:(double)value{
    UISlider *slider=_sliders[key];
    if(!slider || !isfinite(value) || fabs(value)>FLT_MAX) return;
    float sliderValue=(float)value;
    BOOL isGPSControl=SBIsGPSControlKey(key);
    if(sliderValue<slider.minimumValue) slider.minimumValue=isGPSControl?sliderValue:sliderValue-1.0f;
    if(sliderValue>slider.maximumValue) slider.maximumValue=isGPSControl?sliderValue:sliderValue+1.0f;
}
- (void)changed:(UISlider*)s{ NSString *k=s.accessibilityIdentifier; ((UILabel*)_valueLabels[k]).text=SBFormatSettingValue(k,s.value); [self setCur:s.value key:k]; }
- (void)axisChanged:(UISlider*)s{ NSString *k=s.accessibilityIdentifier; ((UILabel*)_valueLabels[k]).text=[NSString stringWithFormat:@"%.2f",s.value]; NSArray *keys=sensorKeys(); NSArray *ax=axisLetters(); for(int si=0;si<S_COUNT;si++) for(int a=0;a<3;a++) if([[keys[si] stringByAppendingString:ax[a]] isEqualToString:k]){ gAxis[si][a]=s.value; return; } }
- (void)tapValue:(UITapGestureRecognizer*)t{
    UILabel *lbl=(UILabel*)t.view; NSString *key=lbl.accessibilityIdentifier;
    UIAlertController *alert=[UIAlertController alertControllerWithTitle:[NSString stringWithFormat:@"Set %@",key] message:@"GPS values accept any finite number, including values outside the slider range. Other sensor entries retain their existing range." preferredStyle:UIAlertControllerStyleAlert];
    [alert addTextFieldWithConfigurationHandler:^(UITextField *tf){ tf.keyboardType=UIKeyboardTypeNumbersAndPunctuation; tf.placeholder=lbl.text; tf.text=lbl.text; }];
    [alert addAction:[UIAlertAction actionWithTitle:@"Cancel" style:UIAlertActionStyleCancel handler:nil]];
    [alert addAction:[UIAlertAction actionWithTitle:@"Set" style:UIAlertActionStyleDefault handler:^(UIAlertAction *a){
        NSString *txt=alert.textFields.firstObject.text; double v=[txt doubleValue];
        UISlider *sl=self->_sliders[key]; if(!sl || !isfinite(v)) return;
        BOOL isGPSControl=SBIsGPSControlKey(key);
        if(!isGPSControl){ if(v<-1000) v=-1000; if(v>1000) v=1000; }
        [self expandSliderForKey:key value:v];
        if(fabs(v)<=FLT_MAX) sl.value=(float)v;
        ((UILabel*)self->_valueLabels[key]).text=SBFormatSettingValue(key,v);
        if([key length]>1 && ([sensorKeys() containsObject:[key substringToIndex:key.length-1]] || [key hasSuffix:@"X"]||[key hasSuffix:@"Y"]||[key hasSuffix:@"Z"])){
            NSArray *keys=sensorKeys(); NSArray *ax=axisLetters(); for(int si=0;si<S_COUNT;si++) for(int p=0;p<3;p++) if([[keys[si] stringByAppendingString:ax[p]] isEqualToString:key]){ gAxis[si][p]=v; return; }
        }
        [self setCur:v key:key];
    }]];
    [_panelWindow.rootViewController presentViewController:alert animated:YES completion:nil];
}
- (UIView*)sensorSectionForIdx:(int)idx{
    NSString *label=sensorLabels()[idx]; NSString *key=sensorKeys()[idx];
    UIView *sec=[UIView new]; sec.layer.cornerRadius=10; sec.backgroundColor=[UIColor secondarySystemGroupedBackgroundColor]; sec.layer.borderWidth=0.5; sec.layer.borderColor=[UIColor separatorColor].CGColor; sec.layer.masksToBounds=YES;
    // Simple grouped header; use system colors so the panel adapts to light and dark mode.
    UIView *header=[UIView new]; header.backgroundColor=[UIColor clearColor];
    UILabel *title=[UILabel new]; title.text=label; title.font=[UIFont boldSystemFontOfSize:13]; title.userInteractionEnabled=YES;
    UILabel *hint=[UILabel new]; hint.text=gSensorAdv[idx]?@"  ▼ 3-axis":@"  ▶︎ tap to expand 3-axis"; hint.font=[UIFont systemFontOfSize:10]; hint.textColor=[UIColor secondaryLabelColor];
    UISwitch *sw=[UISwitch new]; sw.on=gSensorEnabled[idx]; sw.tag=idx; sw.transform=CGAffineTransformMakeScale(0.7,0.7); [sw addTarget:self action:@selector(sensorToggle:) forControlEvents:UIControlEventValueChanged];
    UIStackView *headStack=[[UIStackView alloc] initWithArrangedSubviews:@[title,hint,sw]]; headStack.axis=UILayoutConstraintAxisHorizontal; headStack.spacing=6; headStack.alignment=UIStackViewAlignmentCenter;
    headStack.translatesAutoresizingMaskIntoConstraints=NO; [header addSubview:headStack];
    [NSLayoutConstraint activateConstraints:@[[headStack.topAnchor constraintEqualToAnchor:header.topAnchor constant:8],[headStack.leadingAnchor constraintEqualToAnchor:header.leadingAnchor constant:10],[headStack.trailingAnchor constraintEqualToAnchor:header.trailingAnchor constant:-10],[headStack.bottomAnchor constraintEqualToAnchor:header.bottomAnchor constant:-8]]];
    UITapGestureRecognizer *tapTitle=[[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(headerTapped:)];
    [header addGestureRecognizer:tapTitle]; header.tag=idx; _sensorHeaders[key]=header;

    // absolute row
    UIView *absRow=[self rowForKey:key label:@"All axes" min:-100 max:100];
    _absRows[key]=absRow;
    absRow.hidden=gSensorAdv[idx];

    // adv container
    UIView *advCont=[UIView new];
    UIStackView *advStack=[UIStackView new]; advStack.axis=UILayoutConstraintAxisVertical; advStack.spacing=6;
    for(int a=0;a<3;a++) [advStack addArrangedSubview:[self axisRowForSensor:idx axis:a name:label]];
    advStack.translatesAutoresizingMaskIntoConstraints=NO; [advCont addSubview:advStack];
    [NSLayoutConstraint activateConstraints:@[[advStack.topAnchor constraintEqualToAnchor:advCont.topAnchor constant:6],[advStack.leadingAnchor constraintEqualToAnchor:advCont.leadingAnchor constant:6],[advStack.trailingAnchor constraintEqualToAnchor:advCont.trailingAnchor constant:-6],[advStack.bottomAnchor constraintEqualToAnchor:advCont.bottomAnchor constant:-6]]];
    _advContainers[key]=advCont; advCont.hidden=!gSensorAdv[idx];

    // live readout
    UILabel *live=[UILabel new]; live.font=[UIFont monospacedDigitSystemFontOfSize:9 weight:UIFontWeightRegular]; live.textColor=[UIColor secondaryLabelColor]; live.numberOfLines=2; live.minimumScaleFactor=0.75; live.adjustsFontSizeToFitWidth=YES; live.textAlignment=NSTextAlignmentCenter; live.text=@"live: —";
    _liveLabels[key]=live;

    UIStackView *secStack=[[UIStackView alloc] initWithArrangedSubviews:@[header,absRow,advCont,live]]; secStack.axis=UILayoutConstraintAxisVertical; secStack.spacing=0;
    secStack.translatesAutoresizingMaskIntoConstraints=NO; [sec addSubview:secStack];
    [NSLayoutConstraint activateConstraints:@[[secStack.topAnchor constraintEqualToAnchor:sec.topAnchor],[secStack.leadingAnchor constraintEqualToAnchor:sec.leadingAnchor],[secStack.trailingAnchor constraintEqualToAnchor:sec.trailingAnchor],[secStack.bottomAnchor constraintEqualToAnchor:sec.bottomAnchor]]];
    return sec;
}
- (void)headerTapped:(UITapGestureRecognizer*)t{ int idx=(int)t.view.tag; gSensorAdv[idx]=!gSensorAdv[idx]; NSString *key=sensorKeys()[idx]; UIView *abs=_absRows[key]; UIView *adv=_advContainers[key]; abs.hidden=gSensorAdv[idx]; adv.hidden=!gSensorAdv[idx]; UILabel *hint=nil; for(UIView *v in ((UIStackView*)t.view.subviews.firstObject).arrangedSubviews) if([v isKindOfClass:[UILabel class]] && [((UILabel*)v).text containsString:@"tap"] ) hint=(UILabel*)v; else if([v isKindOfClass:[UILabel class]] && ([((UILabel*)v).text hasPrefix:@"  ▼"] || [((UILabel*)v).text hasPrefix:@"  ▶"])) hint=(UILabel*)v; if(hint) hint.text=gSensorAdv[idx]?@"  ▼ 3-axis":@"  ▶︎ tap to expand 3-axis"; }
- (void)sensorToggle:(UISwitch*)s{ gSensorEnabled[s.tag]=s.isOn; }
- (void)extraToggle:(UISwitch*)s{ gExtraEnabled[s.tag]=s.isOn; }
- (UIView*)extraRowForKey:(NSString*)key label:(NSString*)lab idx:(int)eIdx{
    UIView *row=[UIView new]; row.backgroundColor=[UIColor secondarySystemGroupedBackgroundColor]; row.layer.cornerRadius=10; row.layer.borderWidth=0.5; row.layer.borderColor=[UIColor separatorColor].CGColor; row.layer.masksToBounds=YES;
    UIView *abs=[self rowForKey:key label:@"Scale" min:-100 max:100];
    UISwitch *sw=[UISwitch new]; sw.on=gExtraEnabled[eIdx]; sw.tag=eIdx; sw.transform=CGAffineTransformMakeScale(0.7,0.7); [sw addTarget:self action:@selector(extraToggle:) forControlEvents:UIControlEventValueChanged];
    UILabel *title=[UILabel new]; title.text=lab; title.font=[UIFont boldSystemFontOfSize:13];
    UIStackView *head=[[UIStackView alloc] initWithArrangedSubviews:@[title,[[UIView new] init],sw]]; head.axis=UILayoutConstraintAxisHorizontal;
    UILabel *live=[UILabel new]; live.font=[UIFont monospacedDigitSystemFontOfSize:9 weight:UIFontWeightRegular]; live.textColor=[UIColor secondaryLabelColor]; live.text=@"live: —"; live.textAlignment=NSTextAlignmentCenter; _liveLabels[key]=live;
    UIStackView *stack=[[UIStackView alloc] initWithArrangedSubviews:@[head,abs,live]]; stack.axis=UILayoutConstraintAxisVertical; stack.spacing=6; stack.translatesAutoresizingMaskIntoConstraints=NO;
    [row addSubview:stack]; [NSLayoutConstraint activateConstraints:@[[stack.topAnchor constraintEqualToAnchor:row.topAnchor constant:8],[stack.leadingAnchor constraintEqualToAnchor:row.leadingAnchor constant:10],[stack.trailingAnchor constraintEqualToAnchor:row.trailingAnchor constant:-10],[stack.bottomAnchor constraintEqualToAnchor:row.bottomAnchor constant:-8]]];
    return row;
}
- (UIView*)usageOptionRowWithTitle:(NSString*)title detail:(NSString*)detail option:(int)option{
    UIView *row=[UIView new]; row.backgroundColor=[UIColor secondarySystemGroupedBackgroundColor]; row.layer.cornerRadius=10; row.layer.borderWidth=0.5; row.layer.borderColor=[UIColor separatorColor].CGColor; row.layer.masksToBounds=YES;
    UILabel *heading=[UILabel new]; heading.text=title; heading.font=[UIFont systemFontOfSize:13 weight:UIFontWeightMedium]; heading.numberOfLines=0;
    UILabel *explanation=[UILabel new]; explanation.text=detail; explanation.font=[UIFont systemFontOfSize:10]; explanation.textColor=[UIColor secondaryLabelColor]; explanation.numberOfLines=0;
    UIStackView *copy=[[UIStackView alloc] initWithArrangedSubviews:@[heading,explanation]]; copy.axis=UILayoutConstraintAxisVertical; copy.spacing=3;
    UISwitch *sw=[UISwitch new]; sw.on=gUsageMaskOption[option]; sw.tag=option; sw.transform=CGAffineTransformMakeScale(0.72,0.72); [sw addTarget:self action:@selector(usageMaskToggle:) forControlEvents:UIControlEventValueChanged];
    _usageSwitches[[NSString stringWithFormat:@"%d",option]]=sw;
    UIStackView *line=[[UIStackView alloc] initWithArrangedSubviews:@[copy,sw]]; line.axis=UILayoutConstraintAxisHorizontal; line.spacing=8; line.alignment=UIStackViewAlignmentCenter; line.translatesAutoresizingMaskIntoConstraints=NO;
    [row addSubview:line];
    [NSLayoutConstraint activateConstraints:@[[line.topAnchor constraintEqualToAnchor:row.topAnchor constant:8],[line.leadingAnchor constraintEqualToAnchor:row.leadingAnchor constant:10],[line.trailingAnchor constraintEqualToAnchor:row.trailingAnchor constant:-6],[line.bottomAnchor constraintEqualToAnchor:row.bottomAnchor constant:-8]]];
    return row;
}
- (void)buildCard{
    UILabel *t=[UILabel new]; t.text=@"SensorBender"; t.font=[UIFont systemFontOfSize:22 weight:UIFontWeightSemibold]; t.textAlignment=NSTextAlignmentLeft; [_stack addArrangedSubview:t];
    UILabel *sub=[UILabel new]; sub.text=@"Motion and location controls"; sub.font=[UIFont systemFontOfSize:13]; sub.textColor=[UIColor secondaryLabelColor]; sub.numberOfLines=0; sub.textAlignment=NSTextAlignmentLeft; [_stack addArrangedSubview:sub];
    UIStackView *enRow=[UIStackView new]; enRow.axis=UILayoutConstraintAxisHorizontal; enRow.spacing=8;
    UILabel *enL=[UILabel new]; enL.text=@"Enable for this app"; enL.font=[UIFont systemFontOfSize:15 weight:UIFontWeightMedium];
    _enabledSwitch=[UISwitch new]; _enabledSwitch.on=gAppEnabled; [_enabledSwitch addTarget:self action:@selector(masterToggle:) forControlEvents:UIControlEventValueChanged];
    UIView *sp=[UIView new]; [sp setContentHuggingPriority:UILayoutPriorityDefaultLow forAxis:UILayoutConstraintAxisHorizontal];
    [enRow addArrangedSubview:enL]; [enRow addArrangedSubview:sp]; [enRow addArrangedSubview:_enabledSwitch]; [_stack addArrangedSubview:enRow];
    UIView *sep=[UIView new]; sep.backgroundColor=[UIColor separatorColor]; [sep.heightAnchor constraintEqualToConstant:1].active=YES; [_stack addArrangedSubview:sep];
    [_stack addArrangedSubview:[self section:@"Motion sensors"]];
    for(int i=0;i<3;i++) [_stack addArrangedSubview:[self sensorSectionForIdx:i]];
    [_stack addArrangedSubview:[self section:@"Device motion"]];
    for(int i=3;i<8;i++) [_stack addArrangedSubview:[self sensorSectionForIdx:i]];
    [_stack addArrangedSubview:[self section:@"Barometer"]];
    [_stack addArrangedSubview:[self extraRowForKey:@"pressure" label:@"Pressure" idx:E_PRESSURE]];
    [_stack addArrangedSubview:[self extraRowForKey:@"relAlt" label:@"Relative Alt" idx:E_RELALT]];
    [_stack addArrangedSubview:[self section:@"Location"]];
    UIStackView *gpsEnabledRow=[UIStackView new]; gpsEnabledRow.axis=UILayoutConstraintAxisHorizontal; gpsEnabledRow.spacing=8;
    UILabel *gpsEnabledTitle=[UILabel new]; gpsEnabledTitle.text=@"Enable GPS adjustments"; gpsEnabledTitle.font=[UIFont boldSystemFontOfSize:13];
    UIView *gpsSpacer=[UIView new]; [gpsSpacer setContentHuggingPriority:UILayoutPriorityDefaultLow forAxis:UILayoutConstraintAxisHorizontal];
    _gpsAdjustmentsSwitch=[UISwitch new]; _gpsAdjustmentsSwitch.on=gGPSAdjustmentsEnabled;
    [_gpsAdjustmentsSwitch addTarget:self action:@selector(gpsAdjustmentsToggle:) forControlEvents:UIControlEventValueChanged];
    [gpsEnabledRow addArrangedSubview:gpsEnabledTitle]; [gpsEnabledRow addArrangedSubview:gpsSpacer]; [gpsEnabledRow addArrangedSubview:_gpsAdjustmentsSwitch];
    [_stack addArrangedSubview:gpsEnabledRow];
    [_stack addArrangedSubview:[self rowForKey:@"gpsSmoothing" label:@"Jitter smoothing" min:0 max:0.95]];
    [_stack addArrangedSubview:[self rowForKey:@"gpsSpeed" label:@"GPS speed scale" min:0.1 max:1.0]];
    UILabel *gpsHint=[UILabel new]; gpsHint.text=@"Positive smoothing increases filtering; negative values amplify changes. Speed scale multiplies the adjusted path: 0 pauses it, negative values reverse it, and values above 1 speed it up. This is separate from acceleration scaling."; gpsHint.font=[UIFont systemFontOfSize:10]; gpsHint.textColor=[UIColor secondaryLabelColor]; gpsHint.numberOfLines=0; [_stack addArrangedSubview:gpsHint];
    _gpsLiveLabel=[UILabel new]; _gpsLiveLabel.font=[UIFont monospacedDigitSystemFontOfSize:9 weight:UIFontWeightRegular]; _gpsLiveLabel.textColor=[UIColor secondaryLabelColor]; _gpsLiveLabel.numberOfLines=0; _gpsLiveLabel.text=@"GPS check: waiting for Core Location reads"; [_stack addArrangedSubview:_gpsLiveLabel];
    [_stack addArrangedSubview:[self section:@"Usage privacy · best effort"]];
    UILabel *targetInfo=[UILabel new]; targetInfo.text=[NSString stringWithFormat:@"Settings target (detected from this injected app): %@",gAppBundleID.length?gAppBundleID:@"unknown bundle ID"]; targetInfo.font=[UIFont monospacedDigitSystemFontOfSize:9 weight:UIFontWeightRegular]; targetInfo.textColor=[UIColor secondaryLabelColor]; targetInfo.numberOfLines=0; [_stack addArrangedSubview:targetInfo];
    [_stack addArrangedSubview:[self usageOptionRowWithTitle:@"Hide app usage state" detail:@"When enabled, selected public state queries report active even after this app is backgrounded. iOS may still suspend it or deliver other signals." option:U_MASK]];
    [_stack addArrangedSubview:[self usageOptionRowWithTitle:@"UIApplication reports Active" detail:@"Masks this process's applicationState getter; the overlay uses actual state internally." option:U_APP_STATE]];
    [_stack addArrangedSubview:[self usageOptionRowWithTitle:@"Scenes report Foreground Active" detail:@"Masks UIScene activationState queries for scenes in this app." option:U_SCENE_STATE]];
    [_stack addArrangedSubview:[self usageOptionRowWithTitle:@"Protected data reports available" detail:@"Masks the availability query only; it does not unlock files, the Keychain, or the device." option:U_PROTECTED_DATA]];
    [_stack addArrangedSubview:[self usageOptionRowWithTitle:@"Screen brightness reports 100%" detail:@"Changes the brightness getter result, not the physical display or lock state." option:U_SCREEN_BRIGHTNESS]];
    [_stack addArrangedSubview:[self usageOptionRowWithTitle:@"Suppress background notifications" detail:@"Suppresses selected public background/lock-related NotificationCenter broadcasts. Direct delegate callbacks and private APIs may still reveal state." option:U_LIFECYCLE_NOTIFICATIONS]];
    // Button controls
    [_stack addArrangedSubview:[self section:@"Floating button"]];
    UIView *btnRow1=[self rowForKey:@"btnOpacity" label:@"Opacity" min:0.3 max:1.0];
    UIView *btnRow2=[self rowForKey:@"btnSize" label:@"Size" min:36 max:72];
    // override those sliders to control button
    ((UISlider*)_sliders[@"btnOpacity"]).value=gBtnOpacity; ((UILabel*)_valueLabels[@"btnOpacity"]).text=[NSString stringWithFormat:@"%.2f",gBtnOpacity];
    ((UISlider*)_sliders[@"btnSize"]).value=gBtnSize; ((UILabel*)_valueLabels[@"btnSize"]).text=[NSString stringWithFormat:@"%.0f",gBtnSize];
    [_stack addArrangedSubview:btnRow1]; [_stack addArrangedSubview:btnRow2];
    // Save
    UIButton *save=[UIButton buttonWithType:UIButtonTypeSystem]; [save setTitle:@"Save changes" forState:UIControlStateNormal]; save.titleLabel.font=[UIFont systemFontOfSize:15 weight:UIFontWeightSemibold]; save.backgroundColor=[UIColor systemBlueColor]; [save setTitleColor:[UIColor whiteColor] forState:UIControlStateNormal]; save.layer.cornerRadius=10; [save.heightAnchor constraintEqualToConstant:44].active=YES; [save addTarget:self action:@selector(saveTap) forControlEvents:UIControlEventTouchUpInside]; [_stack addArrangedSubview:save];
    UIStackView *btns=[UIStackView new]; btns.axis=UILayoutConstraintAxisHorizontal; btns.spacing=8; btns.distribution=UIStackViewDistributionFillEqually;
    UIButton *rst=[UIButton buttonWithType:UIButtonTypeSystem]; [rst setTitle:@"Reset defaults" forState:UIControlStateNormal]; rst.backgroundColor=[UIColor secondarySystemFillColor]; rst.layer.cornerRadius=8; [rst.heightAnchor constraintEqualToConstant:40].active=YES; [rst addTarget:self action:@selector(resetTap) forControlEvents:UIControlEventTouchUpInside];
    UIButton *hide=[UIButton buttonWithType:UIButtonTypeSystem]; [hide setTitle:@"Close" forState:UIControlStateNormal]; hide.backgroundColor=[UIColor secondarySystemFillColor]; hide.layer.cornerRadius=8; [hide addTarget:self action:@selector(togglePanel) forControlEvents:UIControlEventTouchUpInside];
    [btns addArrangedSubview:rst]; [btns addArrangedSubview:hide]; [_stack addArrangedSubview:btns];
    UIButton *rm=[UIButton buttonWithType:UIButtonTypeSystem]; [rm setTitle:@"Hide floating button" forState:UIControlStateNormal]; rm.titleLabel.font=[UIFont systemFontOfSize:11]; [rm setTitleColor:[UIColor secondaryLabelColor] forState:UIControlStateNormal]; [rm addTarget:self action:@selector(hideFloat) forControlEvents:UIControlEventTouchUpInside]; [_stack addArrangedSubview:rm];
}
- (void)masterToggle:(UISwitch*)s{ gAppEnabled=s.isOn; SBResetGPSFilter(); }
- (void)gpsAdjustmentsToggle:(UISwitch*)s{ gGPSAdjustmentsEnabled=s.isOn; SBResetGPSFilter(); }
- (void)usageMaskToggle:(UISwitch*)s{ if(s.tag>=0 && s.tag<U_COUNT) gUsageMaskOption[s.tag]=s.isOn; }
- (void)saveTap{
    gBtnOpacity=((UISlider*)_sliders[@"btnOpacity"]).value; gBtnSize=((UISlider*)_sliders[@"btnSize"]).value;
    _floatBtn.alpha=gBtnOpacity; _floatBtn.backgroundColor=[UIColor systemBlueColor];
    CGRect f=_floatBtn.frame; f.size=CGSizeMake(gBtnSize,gBtnSize); _floatBtn.layer.cornerRadius=gBtnSize/2; _floatBtn.frame=f;
    SBResetGPSFilter();
    savePrefs();
    UIImpactFeedbackGenerator *g=[[UIImpactFeedbackGenerator alloc] initWithStyle:UIImpactFeedbackStyleMedium]; [g impactOccurred]; [self togglePanel];
}
- (void)resetTap{
    for(NSString *k in _sliders.allKeys){
        if([k isEqualToString:@"btnOpacity"]||[k isEqualToString:@"btnSize"]) continue;
        float resetValue=[k isEqualToString:@"gpsSmoothing"]?0.65f:1.0f;
        UISlider *sl=(UISlider*)_sliders[k];
        if([k isEqualToString:@"gpsSmoothing"]){ sl.minimumValue=0.0; sl.maximumValue=0.95; }
        else if([k isEqualToString:@"gpsSpeed"]){ sl.minimumValue=0.1; sl.maximumValue=1.0; }
        sl.value=resetValue; ((UILabel*)_valueLabels[k]).text=[NSString stringWithFormat:@"%.2f",resetValue];
        NSArray *keys=sensorKeys(); NSArray *ax=axisLetters(); BOOL isAxis=NO;
        for(int si=0;si<S_COUNT;si++) for(int a=0;a<3;a++) if([[keys[si] stringByAppendingString:ax[a]] isEqualToString:k]){ gAxis[si][a]=1; isAxis=YES; }
        if(!isAxis) [self setCur:resetValue key:k];
    }
    gAppEnabled=YES; _enabledSwitch.on=YES;
    gGPSAdjustmentsEnabled=NO; _gpsAdjustmentsSwitch.on=NO; SBResetGPSFilter();
    gUsageMaskOption[U_MASK]=NO; gUsageMaskOption[U_APP_STATE]=YES; gUsageMaskOption[U_SCENE_STATE]=YES;
    gUsageMaskOption[U_PROTECTED_DATA]=YES; gUsageMaskOption[U_SCREEN_BRIGHTNESS]=NO; gUsageMaskOption[U_LIFECYCLE_NOTIFICATIONS]=NO;
    for(int i=0;i<U_COUNT;i++){ UISwitch *usageSwitch=(UISwitch*)_usageSwitches[[NSString stringWithFormat:@"%d",i]]; usageSwitch.on=gUsageMaskOption[i]; }
    savePrefs();
}
- (void)hideFloat{ _floatWindow.hidden=YES; _panelWindow.hidden=YES; }
- (void)refreshUI{
    _enabledSwitch.on=gAppEnabled;
    _gpsAdjustmentsSwitch.on=gGPSAdjustmentsEnabled;
    for(int i=0;i<U_COUNT;i++){ UISwitch *usageSwitch=(UISwitch*)_usageSwitches[[NSString stringWithFormat:@"%d",i]]; usageSwitch.on=gUsageMaskOption[i]; }
    for(NSString *k in _sliders.allKeys){
        double v=[self cur:k]; NSArray *keys=sensorKeys(); NSArray *ax=axisLetters();
        for(int si=0;si<S_COUNT;si++) for(int a=0;a<3;a++) if([[keys[si] stringByAppendingString:ax[a]] isEqualToString:k]) v=gAxis[si][a];
        if([k isEqualToString:@"btnOpacity"]) v=gBtnOpacity; if([k isEqualToString:@"btnSize"]) v=gBtnSize;
        UISlider *sl=(UISlider*)_sliders[k];
        if(isfinite(v)){ if(fabs(v)<=FLT_MAX){ [self expandSliderForKey:k value:v]; sl.value=(float)v; } else sl.value=v<0?sl.minimumValue:sl.maximumValue; }
        ((UILabel*)_valueLabels[k]).text=SBFormatSettingValue(k,v);
    }
    // update per-sensor header hints
    for(int i=0;i<S_COUNT;i++){ NSString *k=sensorKeys()[i]; UIView *hdr=_sensorHeaders[k]; if(!hdr) continue;
        for(UIView *v in hdr.subviews) for(UIView *sv in v.subviews) if([sv isKindOfClass:[UISwitch class]]) ((UISwitch*)sv).on=gSensorEnabled[i];
        // update abs/adv visibility
        ((UIView*)_absRows[k]).hidden=gSensorAdv[i]; ((UIView*)_advContainers[k]).hidden=!gSensorAdv[i];
    }
}
- (void)updateLive{
    if(_panelWindow.hidden) return;
    // accel/gyro live
    for(int i=0;i<S_COUNT;i++){
        NSString *k=sensorKeys()[i]; UILabel *live=(UILabel*)_liveLabels[k]; if(!live) continue;
        NSString *txt=@"";
        if(i==S_ACCEL) txt=[NSString stringWithFormat:@"live: %.2f,%.2f,%.2f → %.2f,%.2f,%.2f", lastAccel.x,lastAccel.y,lastAccel.z, lastAccelSpoof.x,lastAccelSpoof.y,lastAccelSpoof.z];
        else if(i==S_GYRO) txt=[NSString stringWithFormat:@"live: %.2f,%.2f,%.2f → %.2f,%.2f,%.2f", lastGyro.x,lastGyro.y,lastGyro.z, lastGyroSpoof.x,lastGyroSpoof.y,lastGyroSpoof.z];
        else if(i==S_MAG) txt=[NSString stringWithFormat:@"live mag: %.1f,%.1f,%.1f → %.1f,%.1f,%.1f | compass %.0f°→%.0f°", lastMag.x,lastMag.y,lastMag.z,lastMagSpoof.x,lastMagSpoof.y,lastMagSpoof.z,lastCompassHeading,lastCompassHeadingSpoof];
        else if(i==S_MAGCAL) txt=[NSString stringWithFormat:@"live calibrated mag: %.1f,%.1f,%.1f → %.1f,%.1f,%.1f", lastMagCal.field.x,lastMagCal.field.y,lastMagCal.field.z,lastMagCalSpoof.field.x,lastMagCalSpoof.field.y,lastMagCalSpoof.field.z];
        else if(i==S_ATT) txt=[NSString stringWithFormat:@"live: r%.2f p%.2f y%.2f → r%.2f p%.2f y%.2f", lastRoll,lastPitch,lastYaw, lastRollS,lastPitchS,lastYawS];
        else txt=[NSString stringWithFormat:@"live: updating…"];
        live.text=txt;
    }
    if(_gpsLiveLabel){
        NSUInteger reads=__atomic_load_n(&gGPSCoordinateGetterCalls,__ATOMIC_RELAXED);
        NSUInteger fixes=0; BOOL hasFix=NO; CLLocationCoordinate2D raw={0,0}, adjusted={0,0};
        @synchronized([CLLocation class]){
            fixes=gGPSAdjustedFixCount; hasFix=gGPSHasLiveFix;
            raw=gGPSLiveRawCoordinate; adjusted=gGPSLiveAdjustedCoordinate;
        }
        if(!gGPSAdjustmentsEnabled) _gpsLiveLabel.text=[NSString stringWithFormat:@"GPS check: adjustments off · Core Location reads: %lu",(unsigned long)reads];
        else if(reads==0) _gpsLiveLabel.text=@"GPS check: no hooked CLLocation coordinate reads yet";
        else if(!hasFix) _gpsLiveLabel.text=[NSString stringWithFormat:@"GPS check: %lu coordinate reads, but no usable adjusted fix yet",(unsigned long)reads];
        else {
            double east=0,north=0; SBMetersBetweenCoordinates(raw,adjusted,&east,&north);
            _gpsLiveLabel.text=[NSString stringWithFormat:@"GPS check: %lu reads · %lu adjusted fixes\nRaw: %.6f, %.6f\nOutput: %.6f, %.6f · Δ %.1f m",(unsigned long)reads,(unsigned long)fixes,raw.latitude,raw.longitude,adjusted.latitude,adjusted.longitude,hypot(east,north)];
        }
    }
}
@end

static BOOL SBShouldSuppressLifecycleNotification(NSString *name){
    if(!name.length || !SBUsageMaskIsActive() || !gUsageMaskOption[U_LIFECYCLE_NOTIFICATIONS]) return NO;
    static NSSet *suppressedNames=nil;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken,^{
        suppressedNames=[NSSet setWithArray:@[
            UIApplicationWillResignActiveNotification,
            UIApplicationDidEnterBackgroundNotification,
            UIApplicationProtectedDataWillBecomeUnavailable,
            UIScreenBrightnessDidChangeNotification,
            UISceneWillDeactivateNotification,
            UISceneDidEnterBackgroundNotification
        ]];
    });
    return [suppressedNames containsObject:name];
}
static void SBScheduleOverlayActualStateCheck(void){
    dispatch_async(dispatch_get_main_queue(),^{
        if(SBActualApplicationState()==UIApplicationStateBackground) [[SBSensorBenderOverlay shared] reconcileVisibility];
    });
}

%hook NSNotificationCenter
- (void)postNotificationName:(NSNotificationName)name object:(id)object{
    if(SBShouldSuppressLifecycleNotification(name)){ SBScheduleOverlayActualStateCheck(); return; }
    %orig;
}
- (void)postNotificationName:(NSNotificationName)name object:(id)object userInfo:(NSDictionary*)userInfo{
    if(SBShouldSuppressLifecycleNotification(name)){ SBScheduleOverlayActualStateCheck(); return; }
    %orig;
}
- (void)postNotification:(NSNotification*)notification{
    if(SBShouldSuppressLifecycleNotification(notification.name)){ SBScheduleOverlayActualStateCheck(); return; }
    %orig;
}
%end

%ctor{
    loadPrefs();
    SBInstallCLLocationSubclassHooks();
    [[NSNotificationCenter defaultCenter] addObserverForName:UIApplicationDidFinishLaunchingNotification object:nil queue:[NSOperationQueue mainQueue] usingBlock:^(NSNotification *n){ dispatch_after(dispatch_time(DISPATCH_TIME_NOW,(int64_t)(1.0*NSEC_PER_SEC)),dispatch_get_main_queue(),^{ [[SBSensorBenderOverlay shared] reconcileVisibility]; }); }];
    [[NSNotificationCenter defaultCenter] addObserverForName:UIApplicationDidBecomeActiveNotification object:nil queue:[NSOperationQueue mainQueue] usingBlock:^(NSNotification *n){ [[SBSensorBenderOverlay shared] reconcileVisibility]; }];
    [[NSNotificationCenter defaultCenter] addObserverForName:UIApplicationDidEnterBackgroundNotification object:nil queue:[NSOperationQueue mainQueue] usingBlock:^(NSNotification *n){ [[SBSensorBenderOverlay shared] reconcileVisibility]; }];
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW,(int64_t)(1.5*NSEC_PER_SEC)),dispatch_get_main_queue(),^{ if(SBActualApplicationState()!=UIApplicationStateBackground) [[SBSensorBenderOverlay shared] reconcileVisibility]; });
}
