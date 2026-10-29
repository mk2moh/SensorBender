#import <Foundation/Foundation.h>
#import <CoreMotion/CoreMotion.h>
#import <CoreLocation/CoreLocation.h>
#import <UIKit/UIKit.h>
#import <objc/runtime.h>
#import <math.h>

enum { S_ACCEL, S_GYRO, S_MAG, S_USERACCEL, S_GRAVITY, S_ROTRATE, S_MAGCAL, S_ATT, S_COUNT };
enum { E_PRESSURE, E_RELALT, E_COUNT };

static BOOL gEnabled = YES;
static BOOL gExtraEnabled[E_COUNT];
static BOOL gAppEnabled = YES;
static NSString *gAppBundleID = nil;
static BOOL gSensorEnabled[S_COUNT];
static BOOL gSensorAdv[S_COUNT];
static double gBase[S_COUNT];
static double gAxis[S_COUNT][3];
static double mPressure = 1, mRelAlt = 1;
static BOOL gGPSAdjustmentsEnabled = NO;
static double gGPSSmoothing = 0.65;        // 0 = raw fixes; higher values smooth more.
static double gGPSSpeedMultiplier = 1.0;   // Scales each filtered location step.
static double gBtnOpacity = 0.92, gBtnSize = 54;

static NSArray *sensorKeys(){ return @[@"accel",@"gyro",@"mag",@"userAccel",@"gravity",@"rotRate",@"magCal",@"attitude"]; }
static NSArray *sensorLabels(){ return @[@"Accelerometer",@"Gyroscope",@"Magnetometer",@"User Acceleration",@"Gravity",@"Rotation Rate",@"Calibrated Mag",@"Attitude"]; }
static NSArray *axisLetters(){ return @[@"X",@"Y",@"Z"]; }

static NSUserDefaults *prefsDB(){ return [NSUserDefaults standardUserDefaults]; }
static double readD(id source, NSString *k, double def){ id v=[source objectForKey:k]; return v?[v doubleValue]:def; }
static BOOL readB(id source, NSString *k, BOOL def){ id v=[source objectForKey:k]; return v?[v boolValue]:def; }
static inline double M(int s, int a){ return gSensorAdv[s] ? gAxis[s][a] : gBase[s]; }
static BOOL MSShouldShowOverlay(void){ return NSBundle.mainBundle.bundleIdentifier.length > 0; }

// Keep compass headings consistent with the per-axis magnetometer multipliers.
static __thread BOOL gMSRawCompassAxes=NO;
static double MSSpoofCompassHeading(CLHeading *heading, double originalHeading){
    if(!heading || !gEnabled || !gAppEnabled || !gSensorEnabled[S_MAG] || originalHeading<0) return originalHeading;
    double rawX=0, rawY=0;
    gMSRawCompassAxes=YES;
    @try {
        rawX=heading.x;
        rawY=heading.y;
    } @finally {
        gMSRawCompassAxes=NO;
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

static void MSResetGPSFilter(void){
    @synchronized([CLLocation class]) {
        gGPSFilterInitialized = NO;
        gGPSHasFilteredVelocity = NO;
        gGPSHasCourse = NO;
        gGPSLastTimestamp = 0;
        gGPSFilterGeneration++;
    }
}

static CLLocationCoordinate2D MSCoordinateByAddingMeters(CLLocationCoordinate2D origin, double east, double north){
    const double earthRadius = 6371000.0;
    double latRadians = origin.latitude * M_PI / 180.0;
    double cosine = cos(latRadians);
    if (!isfinite(cosine) || fabs(cosine) < 1e-6) return origin;
    double latitude = origin.latitude + (north / earthRadius) * 180.0 / M_PI;
    double longitude = origin.longitude + (east / (earthRadius * cosine)) * 180.0 / M_PI;
    while (longitude > 180.0) longitude -= 360.0;
    while (longitude < -180.0) longitude += 360.0;
    CLLocationCoordinate2D result = CLLocationCoordinate2DMake(latitude, longitude);
    return CLLocationCoordinate2DIsValid(result) ? result : origin;
}

static void MSMetersBetweenCoordinates(CLLocationCoordinate2D from, CLLocationCoordinate2D to, double *east, double *north){
    const double earthRadius = 6371000.0;
    double deltaLongitude = to.longitude - from.longitude;
    if (deltaLongitude > 180.0) deltaLongitude -= 360.0;
    if (deltaLongitude < -180.0) deltaLongitude += 360.0;
    double meanLatitude = (from.latitude + to.latitude) * 0.5 * M_PI / 180.0;
    *east = deltaLongitude * M_PI / 180.0 * earthRadius * cos(meanLatitude);
    *north = (to.latitude - from.latitude) * M_PI / 180.0 * earthRadius;
}

static NSDictionary *MSGPSAdjustmentForLocation(CLLocation *location, CLLocationCoordinate2D rawCoordinate){
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
                MSMetersBetweenCoordinates(gGPSLastRawCoordinate, rawCoordinate, &east, &north);
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
                    double smoothing = fmax(0.0, fmin(0.95, gGPSSmoothing));
                    double alpha = smoothing == 0.0 ? 1.0 : 1.0 - exp(-dt / (0.05 + 4.0 * smoothing));
                    alpha = fmax(0.05, fmin(1.0, alpha));
                    double rawEastVelocity = east / dt;
                    double rawNorthVelocity = north / dt;
                    double previousEastVelocity = gGPSHasFilteredVelocity ? gGPSFilteredEastVelocity : 0.0;
                    double previousNorthVelocity = gGPSHasFilteredVelocity ? gGPSFilteredNorthVelocity : 0.0;
                    gGPSFilteredEastVelocity = alpha * rawEastVelocity + (1.0 - alpha) * previousEastVelocity;
                    gGPSFilteredNorthVelocity = alpha * rawNorthVelocity + (1.0 - alpha) * previousNorthVelocity;
                    gGPSHasFilteredVelocity = YES;

                    double speedScale = fmax(0.1, fmin(1.0, gGPSSpeedMultiplier));
                    double stepEast = gGPSFilteredEastVelocity * dt * speedScale;
                    double stepNorth = gGPSFilteredNorthVelocity * dt * speedScale;
                    adjusted = MSCoordinateByAddingMeters(gGPSSpoofCoordinate, stepEast, stepNorth);
                    double adjustedDistance = hypot(stepEast, stepNorth);
                    adjustedSpeed = adjustedDistance / dt;
                    hasSpeed = YES;
                    if (adjustedDistance > 0.05) {
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
        objc_setAssociatedObject(location, &gGPSAdjustmentAssociationKey, result, OBJC_ASSOCIATION_RETAIN_NONATOMIC);
        return result;
    }
}

static void loadPrefs(void){
    NSUserDefaults *d=prefsDB();
    gAppBundleID=NSBundle.mainBundle.bundleIdentifier?:@"unknown";
    NSDictionary *allApps=[d dictionaryForKey:@"appPrefs"];
    NSDictionary *app=[allApps[gAppBundleID] isKindOfClass:[NSDictionary class]]?allApps[gAppBundleID]:nil;
    id local=app?:d;
    gEnabled=YES;
    gAppEnabled=readB(local,@"appEnabled",YES);
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
    gGPSSmoothing=fmax(0.0,fmin(0.95,readD(local,@"gpsSmoothing",0.65)));
    gGPSSpeedMultiplier=fmax(0.1,fmin(1.0,readD(local,@"gpsSpeed",1.0)));
    gBtnOpacity=readD(local,@"btnOpacity",0.92); gBtnSize=readD(local,@"btnSize",54);
}

static void savePrefs(void){
    NSUserDefaults *d=prefsDB();
    if(!gAppBundleID.length) gAppBundleID=NSBundle.mainBundle.bundleIdentifier?:@"unknown";
    NSMutableDictionary *apps=[[d dictionaryForKey:@"appPrefs"] mutableCopy]?:[NSMutableDictionary dictionary];
    NSMutableDictionary *app=[NSMutableDictionary dictionary];
    NSArray *keys=sensorKeys(); NSArray *ax=axisLetters();
    app[@"appEnabled"]=@(gAppEnabled);
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
        NSString *p=[doc stringByAppendingPathComponent:@"MotionSpoofer.json"];
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
- (CLLocationDirection)x{ CLLocationDirection v=%orig; if(!gMSRawCompassAxes && gEnabled && gAppEnabled && gSensorEnabled[S_MAG]) v*=M(S_MAG,0); return v; }
- (CLLocationDirection)y{ CLLocationDirection v=%orig; if(!gMSRawCompassAxes && gEnabled && gAppEnabled && gSensorEnabled[S_MAG]) v*=M(S_MAG,1); return v; }
- (CLLocationDirection)z{ CLLocationDirection v=%orig; if(!gMSRawCompassAxes && gEnabled && gAppEnabled && gSensorEnabled[S_MAG]) v*=M(S_MAG,2); return v; }
- (CLLocationDirection)magneticHeading{
    CLLocationDirection real=%orig;
    CLLocationDirection spoof=MSSpoofCompassHeading(self,real);
    lastCompassHeading=real; lastCompassHeadingSpoof=spoof;
    return spoof;
}
- (CLLocationDirection)trueHeading{ CLLocationDirection real=%orig; return MSSpoofCompassHeading(self,real); }
%end

%hook CLLocation
- (CLLocationCoordinate2D)coordinate{
    CLLocationCoordinate2D raw=%orig;
    NSDictionary *adjusted=MSGPSAdjustmentForLocation(self,raw);
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
    return raw<0 ? raw : raw*fmax(0.1,fmin(1.0,gGPSSpeedMultiplier));
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
@interface MSPassThroughView : UIView @end
@implementation MSPassThroughView
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

@interface MSMotionOverlay : NSObject
+ (instancetype)shared; - (void)installIfNeeded; - (void)reconcileVisibility; - (void)togglePanel; - (void)createWindow; - (void)scheduleAutoOpenPanel;
@end

@implementation MSMotionOverlay {
    UIWindow *_floatWindow; UIWindow *_panelWindow;
    MSPassThroughView *_rootView; MSPassThroughView *_panelRootView;
    UIButton *_floatBtn; UIView *_panelBG; UIView *_card; UIScrollView *_scroll; UIStackView *_stack;
    NSMutableDictionary *_sliders; NSMutableDictionary *_valueLabels; NSMutableDictionary *_liveLabels;
    NSMutableDictionary *_sensorHeaders; NSMutableDictionary *_absRows; NSMutableDictionary *_advContainers;
    UISwitch *_enabledSwitch; UISwitch *_gpsAdjustmentsSwitch;
    NSTimer *_liveTimer;
    BOOL _didAutoOpenPanel; BOOL _autoOpenScheduled;
    CGPoint _panStart;
}
+ (instancetype)shared{ static MSMotionOverlay *s; static dispatch_once_t t; dispatch_once(&t,^{s=[MSMotionOverlay new];}); return s; }
- (void)installIfNeeded{
    if(!MSShouldShowOverlay() || [UIApplication sharedApplication].applicationState==UIApplicationStateBackground) return;
    if(_floatWindow){
        dispatch_async(dispatch_get_main_queue(),^{
            if(MSShouldShowOverlay() && [UIApplication sharedApplication].applicationState!=UIApplicationStateBackground){
                self->_floatWindow.hidden=NO;
                [self scheduleAutoOpenPanel];
            }
        });
        return;
    }
    dispatch_async(dispatch_get_main_queue(),^{
        if(MSShouldShowOverlay() && [UIApplication sharedApplication].applicationState!=UIApplicationStateBackground && !self->_floatWindow) [self createWindow];
        else if(MSShouldShowOverlay() && self->_floatWindow) [self scheduleAutoOpenPanel];
    });
}
- (void)reconcileVisibility{
    if(MSShouldShowOverlay() && [UIApplication sharedApplication].applicationState!=UIApplicationStateBackground) [self installIfNeeded];
    else dispatch_async(dispatch_get_main_queue(),^{ self->_panelWindow.hidden=YES; self->_floatWindow.hidden=YES; });
}
- (void)scheduleAutoOpenPanel{
    if(_didAutoOpenPanel || _autoOpenScheduled) return;
    _autoOpenScheduled=YES;
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW,(int64_t)(0.45*NSEC_PER_SEC)),dispatch_get_main_queue(),^{
        self->_autoOpenScheduled=NO;
        if(self->_didAutoOpenPanel || !MSShouldShowOverlay() || [UIApplication sharedApplication].applicationState!=UIApplicationStateActive || !self->_panelWindow) return;
        self->_didAutoOpenPanel=YES;
        if(self->_panelWindow.hidden) [self togglePanel];
    });
}
- (UIWindowScene*)findWindowScene{
    UIWindowScene *fallback=nil;
    for(UIScene *scene in [UIApplication sharedApplication].connectedScenes){
        if(![scene isKindOfClass:[UIWindowScene class]]) continue;
        if(scene.activationState==UISceneActivationStateForegroundActive) return (UIWindowScene*)scene;
        if(scene.activationState==UISceneActivationStateForegroundInactive && !fallback) fallback=(UIWindowScene*)scene;
    }
    return fallback;
}
- (void)createWindow{
    if(!MSShouldShowOverlay() || [UIApplication sharedApplication].applicationState==UIApplicationStateBackground) return;
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
    _rootView=[MSPassThroughView new]; _rootView.frame=CGRectMake(0,0,floatFrame.size.width,floatFrame.size.height); _rootView.backgroundColor=[UIColor clearColor];
    vc1.view=_rootView;
    _floatWindow.rootViewController=vc1; [_floatWindow makeKeyAndVisible];

    _floatBtn=[UIButton buttonWithType:UIButtonTypeCustom];
    _floatBtn.frame=CGRectMake(0,0,gBtnSize,gBtnSize); _floatBtn.layer.cornerRadius=gBtnSize/2;
    _floatBtn.backgroundColor=[UIColor colorWithRed:0.12 green:0.65 blue:0.95 alpha:gBtnOpacity];
    _floatBtn.layer.shadowColor=[UIColor blackColor].CGColor; _floatBtn.layer.shadowOpacity=0.3; _floatBtn.layer.shadowRadius=6; _floatBtn.layer.shadowOffset=CGSizeMake(0,3);
    [_floatBtn setTitle:@"SB" forState:UIControlStateNormal]; _floatBtn.titleLabel.font=[UIFont boldSystemFontOfSize:15];
    [_floatBtn addTarget:self action:@selector(togglePanel) forControlEvents:UIControlEventTouchUpInside];
    UIPanGestureRecognizer *pan=[[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(handlePan:)];
    [_floatBtn addGestureRecognizer:pan];
    [_rootView addSubview:_floatBtn];

    // Panel window - full screen but hidden until opened
    _panelWindow.windowLevel=UIWindowLevelAlert+101; _panelWindow.backgroundColor=[UIColor clearColor]; _panelWindow.hidden=YES;
    UIViewController *vc2=[UIViewController new]; vc2.view.backgroundColor=[UIColor clearColor];
    _panelRootView=[MSPassThroughView new]; _panelRootView.frame=[UIScreen mainScreen].bounds; _panelRootView.backgroundColor=[UIColor clearColor];
    vc2.view=_panelRootView;
    _panelWindow.rootViewController=vc2;

    _panelBG=[[UIView alloc] initWithFrame:_panelRootView.bounds];
    _panelBG.autoresizingMask=UIViewAutoresizingFlexibleWidth|UIViewAutoresizingFlexibleHeight;
    _panelBG.backgroundColor=[UIColor colorWithWhite:0 alpha:0.48]; _panelBG.hidden=NO;
    UITapGestureRecognizer *tapBG=[[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(togglePanel)];
    [_panelBG addGestureRecognizer:tapBG];
    [_panelRootView addSubview:_panelBG];

    _card=[[UIView alloc] init]; _card.backgroundColor=[UIColor systemBackgroundColor]; _card.layer.cornerRadius=18; _card.layer.masksToBounds=YES; _card.translatesAutoresizingMaskIntoConstraints=NO;
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
    _stack=[[UIStackView alloc] init]; _stack.axis=UILayoutConstraintAxisVertical; _stack.spacing=10; _stack.translatesAutoresizingMaskIntoConstraints=NO;
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
- (void)togglePanel{ BOOL show=_panelWindow.hidden; if(show){ [self refreshUI]; _panelWindow.hidden=NO; [_panelWindow makeKeyAndVisible]; } else { _panelWindow.hidden=YES; [_floatWindow makeKeyAndVisible]; } _floatBtn.alpha=show?0.3:1.0; }
- (UILabel*)section:(NSString*)t{ UILabel *l=[UILabel new]; l.text=t; l.font=[UIFont boldSystemFontOfSize:11]; l.textColor=[UIColor secondaryLabelColor]; l.numberOfLines=0; return l; }
- (UIView*)rowForKey:(NSString*)key label:(NSString*)lab min:(float)mn max:(float)mx{
    UIView *row=[UIView new];
    UILabel *ttl=[UILabel new]; ttl.text=lab; ttl.font=[UIFont systemFontOfSize:13 weight:UIFontWeightMedium]; ttl.translatesAutoresizingMaskIntoConstraints=NO; [ttl.widthAnchor constraintEqualToConstant:118].active=YES;
    UISlider *sl=[UISlider new]; sl.minimumValue=mn; sl.maximumValue=mx; sl.value=[self cur:key]; sl.accessibilityIdentifier=key; [sl addTarget:self action:@selector(changed:) forControlEvents:UIControlEventValueChanged];
    UILabel *val=[UILabel new]; val.font=[UIFont monospacedDigitSystemFontOfSize:12 weight:UIFontWeightMedium]; val.textColor=[UIColor systemBlueColor]; val.text=[NSString stringWithFormat:@"%.2f",sl.value]; val.translatesAutoresizingMaskIntoConstraints=NO; [val.widthAnchor constraintEqualToConstant:52].active=YES; val.textAlignment=NSTextAlignmentRight; val.userInteractionEnabled=YES; UITapGestureRecognizer *tap=[[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(tapValue:)]; [val addGestureRecognizer:tap]; val.accessibilityIdentifier=key;
    _sliders[key]=sl; _valueLabels[key]=val;
    UIStackView *h=[[UIStackView alloc] initWithArrangedSubviews:@[ttl,sl,val]]; h.axis=UILayoutConstraintAxisHorizontal; h.spacing=8; h.alignment=UIStackViewAlignmentCenter; h.translatesAutoresizingMaskIntoConstraints=NO;
    [row addSubview:h]; [NSLayoutConstraint activateConstraints:@[[h.topAnchor constraintEqualToAnchor:row.topAnchor],[h.leadingAnchor constraintEqualToAnchor:row.leadingAnchor],[h.trailingAnchor constraintEqualToAnchor:row.trailingAnchor],[h.bottomAnchor constraintEqualToAnchor:row.bottomAnchor]]];
    return row;
}
- (UIView*)axisRowForSensor:(int)idx axis:(int)aIdx name:(NSString*)sensorName{
    NSArray *keys=sensorKeys(); NSString *base=keys[idx]; NSArray *ax=axisLetters(); NSString *letter=ax[aIdx];
    NSString *key=[base stringByAppendingString:letter];
    UIView *row=[UIView new]; UILabel *ttl=[UILabel new]; ttl.translatesAutoresizingMaskIntoConstraints=NO; [ttl.widthAnchor constraintEqualToConstant:118].active=YES;
    NSMutableAttributedString *at=[[NSMutableAttributedString alloc] initWithString:[sensorName uppercaseString] attributes:@{NSFontAttributeName:[UIFont systemFontOfSize:8], NSForegroundColorAttributeName:[UIColor secondaryLabelColor]}];
    UIColor *c=aIdx==0?[UIColor systemRedColor]:aIdx==1?[UIColor systemGreenColor]:[UIColor systemBlueColor];
    NSString *dispName=[sensorName isEqualToString:@"Attitude"] ? (@[@"Roll",@"Pitch",@"Yaw"][aIdx]) : letter;
    NSAttributedString *let=[[NSAttributedString alloc] initWithString:[NSString stringWithFormat:@"  %@",dispName] attributes:@{NSFontAttributeName:[UIFont boldSystemFontOfSize:13], NSForegroundColorAttributeName:c}];
    [at appendAttributedString:let]; ttl.attributedText=at;
    UISlider *sl=[UISlider new]; sl.minimumValue=-5; sl.maximumValue=5; sl.value=(float)gAxis[idx][aIdx]; sl.accessibilityIdentifier=key; sl.minimumTrackTintColor=c; [sl addTarget:self action:@selector(axisChanged:) forControlEvents:UIControlEventValueChanged];
    UILabel *val=[UILabel new]; val.font=[UIFont monospacedDigitSystemFontOfSize:12 weight:UIFontWeightMedium]; val.textColor=c; val.text=[NSString stringWithFormat:@"%.2f",sl.value]; val.translatesAutoresizingMaskIntoConstraints=NO; [val.widthAnchor constraintEqualToConstant:52].active=YES; val.textAlignment=NSTextAlignmentRight; val.userInteractionEnabled=YES; UITapGestureRecognizer *tap=[[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(tapValue:)]; [val addGestureRecognizer:tap]; val.accessibilityIdentifier=key;
    _sliders[key]=sl; _valueLabels[key]=val;
    // live label for axis
    UILabel *live=[UILabel new]; live.font=[UIFont systemFontOfSize:9]; live.textColor=[UIColor tertiaryLabelColor]; live.tag=1000+aIdx;
    UIStackView *h=[[UIStackView alloc] initWithArrangedSubviews:@[ttl,sl,val]]; h.axis=UILayoutConstraintAxisHorizontal; h.spacing=8; h.alignment=UIStackViewAlignmentCenter;
    [row addSubview:h]; h.translatesAutoresizingMaskIntoConstraints=NO;
    [NSLayoutConstraint activateConstraints:@[[h.topAnchor constraintEqualToAnchor:row.topAnchor],[h.leadingAnchor constraintEqualToAnchor:row.leadingAnchor],[h.trailingAnchor constraintEqualToAnchor:row.trailingAnchor],[h.bottomAnchor constraintEqualToAnchor:row.bottomAnchor]]];
    return row;
}
- (float)cur:(NSString*)k{
    if([k isEqualToString:@"accel"]) return gBase[S_ACCEL]; if([k isEqualToString:@"gyro"]) return gBase[S_GYRO]; if([k isEqualToString:@"mag"]) return gBase[S_MAG];
    if([k isEqualToString:@"userAccel"]) return gBase[S_USERACCEL]; if([k isEqualToString:@"gravity"]) return gBase[S_GRAVITY]; if([k isEqualToString:@"rotRate"]) return gBase[S_ROTRATE];
    if([k isEqualToString:@"magCal"]) return gBase[S_MAGCAL]; if([k isEqualToString:@"attitude"]) return gBase[S_ATT];
    if([k isEqualToString:@"pressure"]) return (float)mPressure; if([k isEqualToString:@"relAlt"]) return (float)mRelAlt;
    if([k isEqualToString:@"gpsSmoothing"]) return (float)gGPSSmoothing; if([k isEqualToString:@"gpsSpeed"]) return (float)gGPSSpeedMultiplier;
    NSArray *keys=sensorKeys(); NSArray *ax=axisLetters(); for(int s=0;s<S_COUNT;s++) for(int a=0;a<3;a++) if([[keys[s] stringByAppendingString:ax[a]] isEqualToString:k]) return (float)gAxis[s][a];
    return 1;
}
- (void)setCur:(float)v key:(NSString*)k{
    if([k isEqualToString:@"accel"]) gBase[S_ACCEL]=v; else if([k isEqualToString:@"gyro"]) gBase[S_GYRO]=v; else if([k isEqualToString:@"mag"]) gBase[S_MAG]=v;
    else if([k isEqualToString:@"userAccel"]) gBase[S_USERACCEL]=v; else if([k isEqualToString:@"gravity"]) gBase[S_GRAVITY]=v; else if([k isEqualToString:@"rotRate"]) gBase[S_ROTRATE]=v;
    else if([k isEqualToString:@"magCal"]) gBase[S_MAGCAL]=v; else if([k isEqualToString:@"attitude"]) gBase[S_ATT]=v;
    else if([k isEqualToString:@"pressure"]) mPressure=v; else if([k isEqualToString:@"relAlt"]) mRelAlt=v;
    else if([k isEqualToString:@"gpsSmoothing"]) gGPSSmoothing=fmax(0.0,fmin(0.95,v));
    else if([k isEqualToString:@"gpsSpeed"]) gGPSSpeedMultiplier=fmax(0.1,fmin(1.0,v));
}
- (void)changed:(UISlider*)s{ NSString *k=s.accessibilityIdentifier; ((UILabel*)_valueLabels[k]).text=[NSString stringWithFormat:@"%.2f",s.value]; [self setCur:s.value key:k]; }
- (void)axisChanged:(UISlider*)s{ NSString *k=s.accessibilityIdentifier; ((UILabel*)_valueLabels[k]).text=[NSString stringWithFormat:@"%.2f",s.value]; NSArray *keys=sensorKeys(); NSArray *ax=axisLetters(); for(int si=0;si<S_COUNT;si++) for(int a=0;a<3;a++) if([[keys[si] stringByAppendingString:ax[a]] isEqualToString:k]){ gAxis[si][a]=s.value; return; } }
- (void)tapValue:(UITapGestureRecognizer*)t{
    UILabel *lbl=(UILabel*)t.view; NSString *key=lbl.accessibilityIdentifier;
    UIAlertController *alert=[UIAlertController alertControllerWithTitle:[NSString stringWithFormat:@"Set %@",key] message:@"Enter a multiplier or filter value. GPS controls are clamped to their slider ranges." preferredStyle:UIAlertControllerStyleAlert];
    [alert addTextFieldWithConfigurationHandler:^(UITextField *tf){ tf.keyboardType=UIKeyboardTypeNumbersAndPunctuation; tf.placeholder=lbl.text; tf.text=lbl.text; }];
    [alert addAction:[UIAlertAction actionWithTitle:@"Cancel" style:UIAlertActionStyleCancel handler:nil]];
    [alert addAction:[UIAlertAction actionWithTitle:@"Set" style:UIAlertActionStyleDefault handler:^(UIAlertAction *a){
        NSString *txt=alert.textFields.firstObject.text; double v=[txt doubleValue];
        UISlider *sl=self->_sliders[key]; if(!sl) return;
        if([key isEqualToString:@"gpsSmoothing"]) v=fmax(0.0,fmin(0.95,v));
        else if([key isEqualToString:@"gpsSpeed"]) v=fmax(0.1,fmin(1.0,v));
        else { if(v<-1000) v=-1000; if(v>1000) v=1000; }
        if(![key isEqualToString:@"gpsSmoothing"] && ![key isEqualToString:@"gpsSpeed"]){
            if(sl.minimumValue>v) sl.minimumValue=v-1;
            if(sl.maximumValue<v) sl.maximumValue=v+1;
        }
        sl.value=v; ((UILabel*)self->_valueLabels[key]).text=[NSString stringWithFormat:@"%.2f",v];
        if([key length]>1 && ([sensorKeys() containsObject:[key substringToIndex:key.length-1]] || [key hasSuffix:@"X"]||[key hasSuffix:@"Y"]||[key hasSuffix:@"Z"])){
            NSArray *keys=sensorKeys(); NSArray *ax=axisLetters(); for(int si=0;si<S_COUNT;si++) for(int p=0;p<3;p++) if([[keys[si] stringByAppendingString:ax[p]] isEqualToString:key]){ gAxis[si][p]=v; return; }
        }
        [self setCur:v key:key];
    }]];
    [_panelWindow.rootViewController presentViewController:alert animated:YES completion:nil];
}
- (UIView*)sensorSectionForIdx:(int)idx{
    NSString *label=sensorLabels()[idx]; NSString *key=sensorKeys()[idx];
    UIView *sec=[UIView new]; sec.layer.cornerRadius=10; sec.backgroundColor=[UIColor secondarySystemBackgroundColor]; sec.layer.masksToBounds=YES;
    // header
    UIView *header=[UIView new]; header.backgroundColor=[UIColor tertiarySystemFillColor];
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
    UIView *row=[UIView new]; row.backgroundColor=[UIColor secondarySystemBackgroundColor]; row.layer.cornerRadius=10; row.layer.masksToBounds=YES;
    UIView *abs=[self rowForKey:key label:lab min:-100 max:100];
    UISwitch *sw=[UISwitch new]; sw.on=gExtraEnabled[eIdx]; sw.tag=eIdx; sw.transform=CGAffineTransformMakeScale(0.7,0.7); [sw addTarget:self action:@selector(extraToggle:) forControlEvents:UIControlEventValueChanged];
    UILabel *title=[UILabel new]; title.text=lab; title.font=[UIFont boldSystemFontOfSize:13];
    UIStackView *head=[[UIStackView alloc] initWithArrangedSubviews:@[title,[[UIView new] init],sw]]; head.axis=UILayoutConstraintAxisHorizontal;
    UILabel *live=[UILabel new]; live.font=[UIFont monospacedDigitSystemFontOfSize:9 weight:UIFontWeightRegular]; live.textColor=[UIColor secondaryLabelColor]; live.text=@"live: —"; live.textAlignment=NSTextAlignmentCenter; _liveLabels[key]=live;
    UIStackView *stack=[[UIStackView alloc] initWithArrangedSubviews:@[head,abs,live]]; stack.axis=UILayoutConstraintAxisVertical; stack.spacing=6; stack.translatesAutoresizingMaskIntoConstraints=NO;
    [row addSubview:stack]; [NSLayoutConstraint activateConstraints:@[[stack.topAnchor constraintEqualToAnchor:row.topAnchor constant:8],[stack.leadingAnchor constraintEqualToAnchor:row.leadingAnchor constant:10],[stack.trailingAnchor constraintEqualToAnchor:row.trailingAnchor constant:-10],[stack.bottomAnchor constraintEqualToAnchor:row.bottomAnchor constant:-8]]];
    return row;
}
- (void)buildCard{
    UILabel *t=[UILabel new]; t.text=@"SensorBender • Live"; t.font=[UIFont boldSystemFontOfSize:17]; t.textAlignment=NSTextAlignmentCenter; [_stack addArrangedSubview:t];
    UILabel *sub=[UILabel new]; sub.text=@"Per-app motion and GPS controls. Tap sensor titles for X/Y/Z; tap values to type. Drag the MS button."; sub.font=[UIFont systemFontOfSize:10]; sub.textColor=[UIColor secondaryLabelColor]; sub.numberOfLines=0; sub.textAlignment=NSTextAlignmentCenter; [_stack addArrangedSubview:sub];
    UIStackView *enRow=[UIStackView new]; enRow.axis=UILayoutConstraintAxisHorizontal; enRow.spacing=8;
    UILabel *enL=[UILabel new]; enL.text=@"This App's Spoofing Enabled"; enL.font=[UIFont boldSystemFontOfSize:14];
    _enabledSwitch=[UISwitch new]; _enabledSwitch.on=gAppEnabled; [_enabledSwitch addTarget:self action:@selector(masterToggle:) forControlEvents:UIControlEventValueChanged];
    UIView *sp=[UIView new]; [sp setContentHuggingPriority:UILayoutPriorityDefaultLow forAxis:UILayoutConstraintAxisHorizontal];
    [enRow addArrangedSubview:enL]; [enRow addArrangedSubview:sp]; [enRow addArrangedSubview:_enabledSwitch]; [_stack addArrangedSubview:enRow];
    UIView *sep=[UIView new]; sep.backgroundColor=[UIColor separatorColor]; [sep.heightAnchor constraintEqualToConstant:1].active=YES; [_stack addArrangedSubview:sep];
    [_stack addArrangedSubview:[self section:@"PER-APP RAW SENSORS"]];
    for(int i=0;i<3;i++) [_stack addArrangedSubview:[self sensorSectionForIdx:i]];
    [_stack addArrangedSubview:[self section:@"DEVICE MOTION"]];
    for(int i=3;i<8;i++) [_stack addArrangedSubview:[self sensorSectionForIdx:i]];
    [_stack addArrangedSubview:[self section:@"PER-APP BAROMETER"]];
    [_stack addArrangedSubview:[self extraRowForKey:@"pressure" label:@"Pressure" idx:E_PRESSURE]];
    [_stack addArrangedSubview:[self extraRowForKey:@"relAlt" label:@"Relative Alt" idx:E_RELALT]];
    [_stack addArrangedSubview:[self section:@"GPS / LOCATION"]];
    UIStackView *gpsEnabledRow=[UIStackView new]; gpsEnabledRow.axis=UILayoutConstraintAxisHorizontal; gpsEnabledRow.spacing=8;
    UILabel *gpsEnabledTitle=[UILabel new]; gpsEnabledTitle.text=@"Enable GPS adjustments"; gpsEnabledTitle.font=[UIFont boldSystemFontOfSize:13];
    UIView *gpsSpacer=[UIView new]; [gpsSpacer setContentHuggingPriority:UILayoutPriorityDefaultLow forAxis:UILayoutConstraintAxisHorizontal];
    _gpsAdjustmentsSwitch=[UISwitch new]; _gpsAdjustmentsSwitch.on=gGPSAdjustmentsEnabled;
    [_gpsAdjustmentsSwitch addTarget:self action:@selector(gpsAdjustmentsToggle:) forControlEvents:UIControlEventValueChanged];
    [gpsEnabledRow addArrangedSubview:gpsEnabledTitle]; [gpsEnabledRow addArrangedSubview:gpsSpacer]; [gpsEnabledRow addArrangedSubview:_gpsAdjustmentsSwitch];
    [_stack addArrangedSubview:gpsEnabledRow];
    [_stack addArrangedSubview:[self rowForKey:@"gpsSmoothing" label:@"Jitter smoothing" min:0 max:0.95]];
    [_stack addArrangedSubview:[self rowForKey:@"gpsSpeed" label:@"GPS speed scale" min:0.1 max:1.0]];
    UILabel *gpsHint=[UILabel new]; gpsHint.text=@"Higher smoothing reduces fix-to-fix bounce. Speed scale slows the reported path, so it can trail the real fixes; it is separate from the accelerometer multiplier."; gpsHint.font=[UIFont systemFontOfSize:10]; gpsHint.textColor=[UIColor secondaryLabelColor]; gpsHint.numberOfLines=0; [_stack addArrangedSubview:gpsHint];
    // Button controls
    [_stack addArrangedSubview:[self section:@"FLOATING BUTTON"]];
    UIView *btnRow1=[self rowForKey:@"btnOpacity" label:@"Opacity" min:0.3 max:1.0];
    UIView *btnRow2=[self rowForKey:@"btnSize" label:@"Size" min:36 max:72];
    // override those sliders to control button
    ((UISlider*)_sliders[@"btnOpacity"]).value=gBtnOpacity; ((UILabel*)_valueLabels[@"btnOpacity"]).text=[NSString stringWithFormat:@"%.2f",gBtnOpacity];
    ((UISlider*)_sliders[@"btnSize"]).value=gBtnSize; ((UILabel*)_valueLabels[@"btnSize"]).text=[NSString stringWithFormat:@"%.0f",gBtnSize];
    [_stack addArrangedSubview:btnRow1]; [_stack addArrangedSubview:btnRow2];
    // Save
    UIButton *save=[UIButton buttonWithType:UIButtonTypeSystem]; [save setTitle:@"💾 Save & Apply (persists)" forState:UIControlStateNormal]; save.titleLabel.font=[UIFont boldSystemFontOfSize:14]; save.backgroundColor=[UIColor systemBlueColor]; [save setTitleColor:[UIColor whiteColor] forState:UIControlStateNormal]; save.layer.cornerRadius=11; [save.heightAnchor constraintEqualToConstant:46].active=YES; [save addTarget:self action:@selector(saveTap) forControlEvents:UIControlEventTouchUpInside]; [_stack addArrangedSubview:save];
    UIStackView *btns=[UIStackView new]; btns.axis=UILayoutConstraintAxisHorizontal; btns.spacing=8; btns.distribution=UIStackViewDistributionFillEqually;
    UIButton *rst=[UIButton buttonWithType:UIButtonTypeSystem]; [rst setTitle:@"Reset Defaults" forState:UIControlStateNormal]; rst.backgroundColor=[UIColor secondarySystemFillColor]; rst.layer.cornerRadius=8; [rst.heightAnchor constraintEqualToConstant:40].active=YES; [rst addTarget:self action:@selector(resetTap) forControlEvents:UIControlEventTouchUpInside];
    UIButton *hide=[UIButton buttonWithType:UIButtonTypeSystem]; [hide setTitle:@"Close" forState:UIControlStateNormal]; hide.backgroundColor=[UIColor secondarySystemFillColor]; hide.layer.cornerRadius=8; [hide addTarget:self action:@selector(togglePanel) forControlEvents:UIControlEventTouchUpInside];
    [btns addArrangedSubview:rst]; [btns addArrangedSubview:hide]; [_stack addArrangedSubview:btns];
    UIButton *rm=[UIButton buttonWithType:UIButtonTypeSystem]; [rm setTitle:@"Hide floating button until next launch" forState:UIControlStateNormal]; rm.titleLabel.font=[UIFont systemFontOfSize:11]; [rm setTitleColor:[UIColor secondaryLabelColor] forState:UIControlStateNormal]; [rm addTarget:self action:@selector(hideFloat) forControlEvents:UIControlEventTouchUpInside]; [_stack addArrangedSubview:rm];
}
- (void)masterToggle:(UISwitch*)s{ gAppEnabled=s.isOn; MSResetGPSFilter(); }
- (void)gpsAdjustmentsToggle:(UISwitch*)s{ gGPSAdjustmentsEnabled=s.isOn; MSResetGPSFilter(); }
- (void)saveTap{
    gBtnOpacity=((UISlider*)_sliders[@"btnOpacity"]).value; gBtnSize=((UISlider*)_sliders[@"btnSize"]).value;
    _floatBtn.alpha=gBtnOpacity; _floatBtn.backgroundColor=[UIColor colorWithRed:0.12 green:0.65 blue:0.95 alpha:gBtnOpacity];
    CGRect f=_floatBtn.frame; f.size=CGSizeMake(gBtnSize,gBtnSize); _floatBtn.layer.cornerRadius=gBtnSize/2; _floatBtn.frame=f;
    savePrefs();
    UIImpactFeedbackGenerator *g=[[UIImpactFeedbackGenerator alloc] initWithStyle:UIImpactFeedbackStyleMedium]; [g impactOccurred]; [self togglePanel];
}
- (void)resetTap{
    for(NSString *k in _sliders.allKeys){
        if([k isEqualToString:@"btnOpacity"]||[k isEqualToString:@"btnSize"]) continue;
        float resetValue=[k isEqualToString:@"gpsSmoothing"]?0.65f:1.0f;
        UISlider *sl=(UISlider*)_sliders[k]; sl.value=resetValue; ((UILabel*)_valueLabels[k]).text=[NSString stringWithFormat:@"%.2f",resetValue];
        NSArray *keys=sensorKeys(); NSArray *ax=axisLetters(); BOOL isAxis=NO;
        for(int si=0;si<S_COUNT;si++) for(int a=0;a<3;a++) if([[keys[si] stringByAppendingString:ax[a]] isEqualToString:k]){ gAxis[si][a]=1; isAxis=YES; }
        if(!isAxis) [self setCur:resetValue key:k];
    }
    gAppEnabled=YES; _enabledSwitch.on=YES;
    gGPSAdjustmentsEnabled=NO; _gpsAdjustmentsSwitch.on=NO; MSResetGPSFilter();
    savePrefs();
}
- (void)hideFloat{ _floatWindow.hidden=YES; _panelWindow.hidden=YES; }
- (void)refreshUI{
    _enabledSwitch.on=gAppEnabled;
    _gpsAdjustmentsSwitch.on=gGPSAdjustmentsEnabled;
    for(NSString *k in _sliders.allKeys){
        float v=[self cur:k]; NSArray *keys=sensorKeys(); NSArray *ax=axisLetters();
        for(int si=0;si<S_COUNT;si++) for(int a=0;a<3;a++) if([[keys[si] stringByAppendingString:ax[a]] isEqualToString:k]) v=(float)gAxis[si][a];
        if([k isEqualToString:@"btnOpacity"]) v=gBtnOpacity; if([k isEqualToString:@"btnSize"]) v=gBtnSize;
        UISlider *sl=(UISlider*)_sliders[k]; sl.value=v; ((UILabel*)_valueLabels[k]).text=[NSString stringWithFormat:@"%.2f",v];
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
    // extra live not implemented
}
@end

%ctor{
    loadPrefs();
    [[NSNotificationCenter defaultCenter] addObserverForName:UIApplicationDidFinishLaunchingNotification object:nil queue:[NSOperationQueue mainQueue] usingBlock:^(NSNotification *n){ dispatch_after(dispatch_time(DISPATCH_TIME_NOW,(int64_t)(1.0*NSEC_PER_SEC)),dispatch_get_main_queue(),^{ [[MSMotionOverlay shared] reconcileVisibility]; }); }];
    [[NSNotificationCenter defaultCenter] addObserverForName:UIApplicationDidBecomeActiveNotification object:nil queue:[NSOperationQueue mainQueue] usingBlock:^(NSNotification *n){ [[MSMotionOverlay shared] reconcileVisibility]; }];
    [[NSNotificationCenter defaultCenter] addObserverForName:UIApplicationDidEnterBackgroundNotification object:nil queue:[NSOperationQueue mainQueue] usingBlock:^(NSNotification *n){ [[MSMotionOverlay shared] reconcileVisibility]; }];
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW,(int64_t)(1.5*NSEC_PER_SEC)),dispatch_get_main_queue(),^{ if([UIApplication sharedApplication].applicationState!=UIApplicationStateBackground) [[MSMotionOverlay shared] reconcileVisibility]; });
}
