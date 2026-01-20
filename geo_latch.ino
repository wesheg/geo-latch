#include <LiquidCrystal.h>
#include <Adafruit_GPS.h>
#include <PWMServo.h>
#define GPSECHO false   


LiquidCrystal lcd (10, 11, 5, 2, 1, 0);                
boolean usingInterrupt = false;           // used for interrupt running in background for reading GPS
void useInterrupt(boolean);
SoftwareSerial mySerial(8, 7);          
Adafruit_GPS GPS(&mySerial);
PWMServo myservo;

const int unlocked = 0;
const int locked = 50;
const int servo = 9;
bool box_locked = false;
const int led = 3;                              // led ring around the red button
const int button = 12;                          // red button input pin
int override_push_count = 0;                    // sets number of button pushes for lock override
int total_push_count = 0;                       // keeps track of total red button pushes
int last_button_state = digitalRead(button);    // integer used in override lock function
unsigned long override_timer_start = 0;         // timer used in override lock function
bool lock_override = false;


const int destination_count = 1;        // number of destinations in the scavenger hunt

// latitude is first; longitude second
const float destinations[destination_count][2] = {
    {36.108867, -86.764774}
  };


const int earth_radius = 3961;            // Earth's radius in miles
const int red_led_ring = 3;               // pin for light on red button 
String row1_last = "";                    // initial variables for lcd_clean_print
String row2_last = "";                    // initial variables for lcd_clean_print


int get_next_destination (float current_lat, float current_long) {
  int next_destination = 0;
  for (int i = 0; i < destination_count; i++) {
    float distance = haversine_distance(current_lat, current_long, 
                                        destinations[i][0], destinations[i][1]);
    if (distance < 0.03) {
      if (i < destination_count - 1) {
        next_destination = i + 1;
      } 

    }
    
  }

  return next_destination;
}


float haversine_distance(float lat1, float long1, float lat2, float long2) {
  // calculates the distance in miles between 2 latitude/longitude coordinates
  // inputs in decimal degrees

  float lat1R = lat1 * PI / 180;
  float long1R = long1 * PI / 180;
  float lat2R = lat2 * PI / 180;
  float long2R = long2 * PI / 180;
  
  float dlat = (lat2 - lat1) * PI / 180;
  float dlon = (long2 - long1) * PI / 180;
  float a = pow(sin(dlat / 2), 2) + cos(lat1R) * cos(lat2R) * pow(sin(dlon / 2), 2);
  float c = 2 * atan2(pow(a, 0.5), pow(1 - a, 0.5));
  float d = earth_radius * c;
  return d;
  
  
  }


/**
 * Calculates the distance in miles between two coordinates on the same longitude.
 * 
 * @param lat1 Latitude of the first coordinate in decimal degrees.
 * @param lat2 Latitude of the second coordinate in decimal degrees.
 * @param long2 Longitude of both coordinates in decimal degrees.
 * @return Distance in miles between the two coordinates.
 */
float haversine_y_distance(float lat1, float lat2, float long2) {
  float lat1R = lat1 * PI / 180;
  float long1R = long2 * PI / 180;
  float lat2R = lat2 * PI / 180;
  float long2R = long2 * PI / 180;
  
  float dlat = (lat2 - lat1) * PI / 180;
  float dlon = 0;
  float a = pow(sin(dlat / 2), 2) + cos(lat1R) * cos(lat2R) * pow(sin(dlon / 2), 2);
  float c = 2 * atan2(pow(a, 0.5), pow(1 - a, 0.5));
  float d = earth_radius * c;
  return d;
}

/**
 * Calculates the distance in miles between two coordinates on the same latitude.
 * 
 * @param long1 Longitude of the first coordinate in decimal degrees.
 * @param long2 Longitude of the second coordinate in decimal degrees.
 * @param lat2 Latitude of both coordinates in decimal degrees.
 * @return Distance in miles between the two coordinates.
 */
float haversine_x_distance(float long1, float long2, float lat2) {
  float lat1R = lat2 * PI / 180;
  float long1R = long1 * PI / 180;
  float lat2R = lat2 * PI / 180;
  float long2R = long2 * PI / 180;
  
  float dlat = 0;
  float dlon = (long1 - long2) * PI / 180;
  float a = pow(sin(dlat / 2), 2) + cos(lat1R) * cos(lat2R) * pow(sin(dlon / 2), 2);
  float c = 2 * atan2(pow(a, 0.5), pow(1 - a, 0.5));
  float d = earth_radius * c;
  return d;
}

/**
 * Converts distance in miles to a string with appropriate units.
 * 
 * @param distance Distance in miles.
 * @return A string representing the distance with units ("miles" or "yards").
 */
String unit_convert(float distance) {
  String new_distance_string;
  String units;
  if (distance >= 1) {
    new_distance_string = String(distance, 1);
    units = " miles";
  } else {
    new_distance_string = String(distance * 1760, 0);
    units = " yards";  
  }
  return new_distance_string + units;
}


/**
 * Determines the cardinal direction from the current coordinate to the destination coordinate.
 * 
 * @param current_coordinate The current latitude or longitude.
 * @param destination_coordinate The destination latitude or longitude.
 * @param latitude A boolean indicating if the coordinates are latitude (true) or longitude (false).
 * @return A string representing the cardinal direction ("North", "South", "East", or "West").
 */
String cardinal_direction(
  float current_coordinate,
  float destination_coordinate,
  bool latitude
){
  if (latitude) {
    if (destination_coordinate > current_coordinate) {
      return "North";
    }
    return "South"; 
  }

  if (destination_coordinate > current_coordinate) {
    return "East";
  }
    return "West";
}


/**
 * Records the number of button pushes within a 5-second window.
 */
void record_button_push() {
  int current_time = millis();
  if (override_push_count == 0 && digitalRead(button) == HIGH) {
    // starts 5 second timer as soon as the first push is complete
    override_timer_start = current_time;
  }
  
  if ((current_time - override_timer_start) >= 5000) {
    // start override_push_count back at 0 if more than 5 seconds have passed
    override_timer_start = current_time;
    override_push_count = 0;
  }

  if (digitalRead(button) == LOW) {
    if (last_button_state == 1) {
      override_push_count += 1;
      total_push_count += 1;
    }
    last_button_state = 0;
    
  } else if (digitalRead(button) == HIGH) {
    last_button_state = 1;
  }  
}


void override_lock() {
  if (digitalRead(button) == HIGH) {
    unlock_box();
    digitalWrite(led, HIGH);
  } else {
    lock_box();
    lock_override = false;
    digitalWrite(led, LOW);
  }
}
  
/**
 * Rotate the servo to unlock the box.
 */
void unlock_box() {
  myservo.write(unlocked);
}

/**
 * Rotate the servo to lock the box.
 */
void lock_box() {
  myservo.write(locked);
}


void red_button () {
  if (digitalRead(button) == LOW) {
    for (int fadeValue = 0; fadeValue <= 255; fadeValue += 17) {
      if (digitalRead(button) == LOW) {
        analogWrite(led, fadeValue);
        lock_box();
        } else {
          digitalWrite(led, HIGH);
          unlock_box();
        }
      
      delay(30);
      }
  
    for (int fadeValue = 255; fadeValue >=0; fadeValue -= 17) {
      if (digitalRead(button) == LOW) {
        analogWrite(led, fadeValue);
        lock_box();
        } else {
          digitalWrite(led, HIGH);
          unlock_box();
        }
      delay(30);
      }
  } else {
    digitalWrite(led, HIGH);
    }
  
}


void lcd_start() {
  // display welcome message
  int timer_start = millis();
  int current_time = millis();
  
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Hello Catie & ");
  lcd.setCursor(2, 1);
  lcd.print("Kaitlynn!");

  while (current_time - timer_start < 5000) {
    current_time = millis();
  }
  
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Ready for an");
  lcd.setCursor(2, 1);
  lcd.print("adventure?");

  while (current_time - timer_start < 10000) {
    current_time = millis();
  }

  lcd.clear();
  lcd.print("    Take Me:    ");

  while (current_time - timer_start < 13000) {
    current_time = millis();
  }

}




void lcd_clean_print(String row1_new, String row2_new) {
  // prints to lcd board without clearing the whole console
  // prevents lcd from blinking
  
  int row1_blanks = row1_new.length() - row1_last.length();
  int row2_blanks = row2_new.length() - row2_last.length();
  int row_blanks[2] = {row1_blanks, row2_blanks};
  String new_rows[2] = {row1_new, row2_new};
  
  
  for (int row = 0; row <= 1; row ++){
    lcd.setCursor(0, row);
    lcd.print(new_rows[row]);
    if (row_blanks[row] < 0) {
      for (int i = 0; i <= abs(row_blanks[row]); i++) {
        lcd.setCursor(new_rows[row].length() + i, row);
        lcd.print(" ");
        }
      
      }
    
    }

    row1_last = row1_new;
    row2_last = row2_new;
  
  
  }

float decimal_degrees (float coordinate, String cardinal) {
  // converts adafruit gps coordinate to decimal degrees
  // use true if latitude, false if longitude
  // adafruit latitude is formatted DDMM.MMMMM, longitude is DDDMM.MMMMM
  // use "N", "S", "E", "W" for cardinal variable; North and East return positive, South and West Return negative
  
  float deg = int(coordinate / 100);
  float dec_deg = (coordinate - (deg * 100)) / 60;
  float decimal_degrees = deg + dec_deg;

  if (cardinal == "N" || cardinal == "E") {
    return decimal_degrees;
  } else if (cardinal == "S" || cardinal == "W") {
    return -decimal_degrees;
  } else {
      return;
    }
  
  }


SIGNAL(TIMER0_COMPA_vect) {
  // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  char c = GPS.read();
  if (GPSECHO)
    if (c) UDR0 = c;  
}
 
void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void setup() {
  pinMode(servo, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(button, INPUT);
  myservo.attach(servo);
  lock_box();
  lcd.begin(16,2);


  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);                          // RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);                             // 1 Hz update rate
  useInterrupt(true);                                                    // reads the steaming data in a background

//  Serial.begin(115200);



  
  
  lcd_start();
  
  
 
  
}



//float destination_lat = destinations[0][0];
//float destination_long = destinations[0][1];

float destination_lat = 0;
float destination_long = 0;

void loop() {

  record_button_push();
//  Serial.println(total_push_count);


  if (override_push_count >= 4 && digitalRead(button) == HIGH) {
    lock_override = true;
  }

  
  if (lock_override) {
    override_lock();
  }
  
  // Parse GPS and recalculate RANGE
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))                                      // also sets the newNMEAreceived() flag to false
      return;                                                            // We can fail to parse a sentence in which case we should just wait for another
  }
    if (GPS.fix) {
//    lcd.clear();

      float current_lat = decimal_degrees(float(GPS.latitude), String(GPS.lat));
      float current_long = decimal_degrees(float(GPS.longitude), String(GPS.lon));

      if (destination_lat == 0 && destination_long == 0) {
        int start = get_next_destination(current_lat, current_long);
        destination_lat = destinations[start][0];
        destination_long = destinations[start][1];
        
      }

      float y = haversine_x_distance(current_long, destination_long, destination_lat);
      float x = haversine_y_distance(current_lat, destination_lat, destination_long);
    
      String x_direction = cardinal_direction(current_lat, destination_lat, true);
      String y_direction = cardinal_direction(current_long, destination_long, false);
    
      String x_units = unit_convert(x);
      String y_units = unit_convert(y);
      
      String output_x = x_units + ' ' + x_direction;
      String output_y = y_units + ' ' + y_direction;
  
      
      if (x < 0.015 && y < 0.015) {   // draw 25 yard radius around destination
          int starting_push_count = total_push_count;
          while (total_push_count == starting_push_count) {
            lcd_clean_print("  Open the box!  ", "");
            red_button();
            record_button_push();
            
          }

          
          lcd.clear();
          if (destination_lat == destinations[destination_count - 1][0] && 
                destination_long == destinations[destination_count - 1][1]) {
             lcd_clean_print("Congratulations!", "");
             delay(10000); 
            
          }
          
          lcd_clean_print("  Finding Next  ", " Destination... ");
          delay(2000);
          int next_destination = get_next_destination(current_lat, current_long);
          destination_lat = destinations[next_destination][0];
          destination_long = destinations[next_destination][1];
          
          

      } else {
        lcd_clean_print(output_x, output_y);
//        Serial.println(output_x);
//        Serial.println(output_y);

//        lcd_clean_print(String(destination_lat, 5), String(destination_long, 5));
        
        }

  }
  else {                                                              //No GPS fix- take box outside
    lcd_clean_print("Take me outside!", "");
  }

}
