#include <Arduino.h>

/* ************************************************************************** */

/* Feature Constants */
#define CLOCK_PCF85063TP 1
#define CLOCK_DS1307 2
#define CLOCK_DS3231 3
#define BATTERY_GAUGE_DFROBOT 1
#define BATTERY_GAUGE_LC709203F 2

/* Software Parameters */
#define DATA_FILE_PATH "/data.csv"
#define CLEANUP_FILE_PATH "/cleanup.csv"
#define LOG_FILE_PATH "/log.csv"
#define SYNCHONIZE_INTERVAL 7654321UL /* milliseconds */
#define SYNCHONIZE_MARGIN 1234UL /* milliseconds */
#define CLEANLOG_INTERVAL 86400000UL /* milliseconds */
#define SLEEP_MARGIN 1000 /* milliseconds */

/* Hardware Parameters */
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ROTATION 2
#define OLED_I2C_ADDR 0x3C
#define LORA_BAND 923000000

/* Protocol Constants */
#define PACKET_TIME    0
#define PACKET_ASKTIME 1
#define PACKET_ACK     2
#define PACKET_SEND    3

#define CIPHER_IV_LENGTH 12
#define CIPHER_TAG_SIZE 4

/* ************************************************************************** */

#include "config.h"

#if !defined(DEVICE_ID)
	#error "ERROR: undefined DEVICE_ID"
#endif

#if !defined(NUMBER_OF_DEVICES)
	#error "ERROR: undefined NUMBER_OF_DEVICES"
#endif

#if !defined(ENABLE_GATEWAY)
	#define CPU_FREQUENCY 20 /* MUST: >= 20 for LoRa, and >= 80 for WiFi */
#endif
#if defined(ENABLE_GATEWAY) && defined(CPU_FREQUENCY) && !(CPU_FREQUENCY >= 80)
	#undef CPU_FREQUENCY
	#define CPU_FREQUENCY 80
#endif
#if defined(CPU_FREQUENCY) && !(CPU_FREQUENCY >= 20)
	#undef CPU_FREQUENCY
	#define CPU_FREQUENCY 20
#endif

#ifndef SEND_INTERVAL
	#define SEND_INTERVAL (ACK_TIMEOUT * (RESEND_TIMES + 2))
#endif

/* ************************************************************************** */

#include <stdlib.h>
#include <time.h>
#include <cstring>
#include <memory>
#include <vector>

#include <RNG.h>
#include <AES.h>
#include <GCM.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <Adafruit_SSD1306.h>

typedef uint8_t PacketType;
typedef unsigned long int Time;
typedef uint8_t Device;
typedef uint32_t SerialNumber;
typedef GCM<AES128> AuthCipher;

static Device const router_topology[][2] = ROUTER_TOPOLOGY;
static char const secret_key[16] = SECRET_KEY;
#if defined(ENABLE_SD_CARD)
	static char const data_file_path[] = DATA_FILE_PATH;
	static char const cleanup_file_path[] = CLEANUP_FILE_PATH;
	static char const log_file_path[] = LOG_FILE_PATH;
#endif

template <typename TYPE>
uint8_t rand_int(void) {
	TYPE x;
	RNG.rand((uint8_t *)&x, sizeof x);
	return x;
}

namespace LED {
	#ifdef ENABLE_LED
		static void initialize(void) {
			pinMode(LED_BUILTIN, OUTPUT);
			digitalWrite(LED_BUILTIN, LOW);
		}

		static void flash(void) {
			static bool light = true;
			digitalWrite(LED_BUILTIN, light ? HIGH : LOW);
			light = !light;
			delay(200);
		}
	#else
		inline static void initialize(void) {}
		inline static void flash(void) {}
	#endif
}

namespace COM {
	#ifdef ENABLE_COM_OUTPUT
		inline static void initialize(void) {
			#ifdef CPU_FREQUENCY
				#if CPU_FREQUENCY < 80
					Serial.begin(COM_BAUD * 80 / CPU_FREQUENCY);
				#else
					Serial.begin(COM_BAUD);
				#endif
			#else
				Serial.begin(COM_BAUD);
			#endif
		}

		template <typename TYPE>
		inline void print(TYPE const x) {
			Serial.print(x);
		}

		template <typename TYPE>
		inline void println(TYPE x) {
			Serial.println(x);
		}

		template <typename TYPE>
		inline void println(TYPE const x, int option) {
			Serial.println(x, option);
		}

		void dump(char const *const label, void const *const memory, size_t const size) {
			Serial.printf("%s (%04X)", label, size);
			for (size_t i = 0; i < size; ++i) {
				unsigned char const c = i[(unsigned char const *)memory];
				Serial.printf(" %02X", c);
			}
			Serial.write('\n');
		}
	#else
		inline static void initialize(void) {
			Serial.end();
		}

		template <typename TYPE> inline void print([[maybe_unused]] TYPE x) {}
		template <typename TYPE> inline void println([[maybe_unused]] TYPE x) {}
		template <typename TYPE> inline void print([[maybe_unused]] TYPE x, [[maybe_unused]] int option) {}
		template <typename TYPE> inline void println([[maybe_unused]] TYPE x, [[maybe_unused]] int option) {}
		inline static void dump(
			[[maybe_unused]] char const *const label,
			[[maybe_unused]] void const *const memory,
			[[maybe_unused]] size_t const size
		) {}
	#endif
}

namespace OLED {
	static Adafruit_SSD1306 SSD1306(OLED_WIDTH, OLED_HEIGHT);

	static void turn_off(void) {
		SSD1306.ssd1306_command(SSD1306_CHARGEPUMP);
		SSD1306.ssd1306_command(0x10);
		SSD1306.ssd1306_command(SSD1306_DISPLAYOFF);
	}

	#if defined(ENABLE_OLED_OUTPUT)
		static class String message;
		static bool switched_off;

		static void turn_on(void) {
			SSD1306.ssd1306_command(SSD1306_CHARGEPUMP);
			SSD1306.ssd1306_command(0x14);
			SSD1306.ssd1306_command(SSD1306_DISPLAYON);
		}

		static void initialize(void) {
			SSD1306.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
			#if defined(ENABLE_OLED_SWITCH)
				pinMode(ENABLE_OLED_SWITCH, INPUT);
				switched_off = false;
			#endif
			SSD1306.invertDisplay(false);
			SSD1306.setRotation(OLED_ROTATION);
			SSD1306.setTextSize(1);
			SSD1306.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
			SSD1306.clearDisplay();
			SSD1306.display();
			SSD1306.setCursor(0, 0);
		}

		static void check_switch(void) {
			#if defined(ENABLE_OLED_SWITCH)
				if (digitalRead(ENABLE_OLED_SWITCH) == LOW) {
					if (!switched_off) {
						turn_off();
						switched_off = true;
					}
				} else {
					if (switched_off) {
						turn_on();
						switched_off = false;
					}
				}
			#endif
		}

		inline static void home(void) {
			SSD1306.clearDisplay();
			SSD1306.setCursor(0, 0);
		}

		template <typename TYPE>
		inline void print(TYPE x) {
			SSD1306.print(x);
		}

		template <typename TYPE>
		inline void println(TYPE x) {
			SSD1306.println(x);
		}

		template <typename TYPE>
		inline void println(TYPE const x, int const option) {
			SSD1306.println(x, option);
		}

		inline static void set_message(class String const &string) {
			message = string;
		}

		inline static void print_message(void) {
			print(message);
		}

		inline static void display(void) {
			SSD1306.display();
		}
	#else
		inline static void initialize(void) {
			SSD1306.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
			turn_off();
		}
		inline static void check_switch(void) {}
		inline static void home(void) {}
		template <typename TYPE> inline void print([[maybe_unused]] TYPE x) {}
		template <typename TYPE> inline void println([[maybe_unused]] TYPE x) {}
		template <typename TYPE> inline void println([[maybe_unused]] TYPE x, [[maybe_unused]] int option) {}
		inline static void set_message([[maybe_unused]] class String const &string) {}
		inline static void print_message(void) {}
		inline static void display(void) {}
	#endif
}

template <typename TYPE>
inline void any_print(TYPE x) {
	COM::print(x);
	OLED::print(x);
}

template <typename TYPE>
inline void any_println(TYPE x) {
	COM::println(x);
	OLED::println(x);
}

template <typename TYPE>
inline void any_println(TYPE x, int option) {
	COM::println(x, option);
	OLED::println(x, option);
}

namespace Debug {
	template <typename TYPE>
	[[maybe_unused]]
	inline void print([[maybe_unused]] TYPE x) {
		#if !defined(NDEBUG)
			COM::print(x);
		#endif
	}

	template <typename TYPE>
	[[maybe_unused]]
	inline void println([[maybe_unused]] TYPE x) {
		#if !defined(NDEBUG)
			COM::println(x);
		#endif
	}

	inline void dump(
		[[maybe_unused]] char const *const label,
		[[maybe_unused]] void const *const memory,
		[[maybe_unused]] size_t const size
	) {
		COM::dump(label, memory, size);
	}

	[[maybe_unused]]
	inline static void flush(void) {
		#if !defined(NDEBUG) && defined(ENABLE_COM_OUTPUT)
			Serial.flush();
		#endif
	}
}

struct [[gnu::packed]] FullTime {
	unsigned short int year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char minute;
	unsigned char second;

	explicit operator String(void) const;
};

FullTime::operator String(void) const {
	char buffer[48];
	snprintf(
		buffer, sizeof buffer,
		"%04u-%02u-%02uT%02u:%02u:%02uZ",
		this->year, this->month, this->day,
		this->hour, this->minute, this->second
	);
	return String(buffer);
}

#if !defined(ENABLE_CLOCK)
	#include <RTClib.h>

	namespace RTC {
		static bool clock_available;
		static class RTC_Millis internal_clock;

		static bool initialize(void) {
			clock_available = false;
			return true;
		}

		static void set(struct FullTime const &fulltime) {
			class DateTime const datetime(
				fulltime.year, fulltime.month, fulltime.day,
				fulltime.hour, fulltime.minute, fulltime.second
			);
			if (clock_available) {
				internal_clock.adjust(datetime);
			} else {
				internal_clock.begin(datetime);
				clock_available = true;
			}
		}

		static bool now(struct FullTime *const fulltime) {
			class DateTime const datetime = internal_clock.now();
			if (fulltime != NULL)
				*fulltime = {
					.year = (unsigned short int)datetime.year(),
					.month = (unsigned char)datetime.month(),
					.day = (unsigned char)datetime.day(),
					.hour = (unsigned char)datetime.hour(),
					.minute = (unsigned char)datetime.minute(),
					.second = (unsigned char)datetime.second()
				};
			return clock_available;
		}
	}
#elif ENABLE_CLOCK == CLOCK_PCF85063TP
	#include <PCF85063TP.h>

	namespace RTC {
		static class PCD85063TP external_clock;

		static bool initialize(void) {
			external_clock.begin();
			external_clock.startClock();
			return true;
		}

		static void set(struct FullTime const &fulltime) {
			external_clock.stopClock();
			external_clock.fillByYMD(fulltime.year, fulltime.month, fulltime.day);
			external_clock.fillByHMS(fulltime.hour, fulltime.minute, fulltime.second);
			external_clock.setTime();
			external_clock.startClock();
		}

		static bool now(struct FullTime *const fulltime) {
			external_clock.getTime();
			if (fulltime != NULL)
				*fulltime = {
					.year = (unsigned short int)(2000U + external_clock.year),
					.month = external_clock.month,
					.day = external_clock.dayOfMonth,
					.hour = external_clock.hour,
					.minute = external_clock.minute,
					.second = external_clock.second
				};
			static bool available = false;
			if (!available)
				available =
					1 <= external_clock.year       && external_clock.year       <= 99 &&
					1 <= external_clock.month      && external_clock.month      <= 12 &&
					1 <= external_clock.dayOfMonth && external_clock.dayOfMonth <= 30 &&
					0 <= external_clock.hour       && external_clock.hour       <= 23 &&
					0 <= external_clock.minute     && external_clock.minute     <= 59 &&
					0 <= external_clock.second     && external_clock.second     <= 59;
			return available;
		}
	}
#elif ENABLE_CLOCK == CLOCK_DS1307 || ENABLE_CLOCK == CLOCK_DS3231
	#include <RTClib.h>

	namespace RTC {
		#if ENABLE_CLOCK == CLOCK_DS1307
			static class RTC_DS1307 external_clock;
		#elif ENABLE_CLOCK == CLOCK_DS3231
			static class RTC_DS3231 external_clock;
		#endif

		static bool initialize(void) {
			if (!external_clock.begin()) {
				any_println("Clock not found");
				return false;
			}
			#if ENABLE_CLOCK == CLOCK_DS1307
				if (!external_clock.isrunning()) {
					any_println("DS1307 not running");
					return false;
				}
			#endif
			return true;
		}

		static void set(struct FullTime const &fulltime) {
			class DateTime const datetime(
				fulltime.year, fulltime.month, fulltime.day,
				fulltime.hour, fulltime.minute, fulltime.second
			);
			external_clock.adjust(datetime);
		}

		static bool now(struct FullTime *const fulltime) {
			class DateTime const datetime = external_clock.now();
			if (fulltime != NULL)
				*fulltime = {
					.year = datetime.year(),
					.month = datetime.month(),
					.day = datetime.day(),
					.hour = datetime.hour(),
					.minute = datetime.minute(),
					.second = datetime.second()
				};
			return datetime.isValid();
		}
	}
#endif

#if defined(ENABLE_GATEWAY)
	#include <NTPClient.h>

	namespace NTP {
		static class WiFiUDP WiFiUDP;
		static class NTPClient NTP(WiFiUDP, NTP_SERVER, 0, NTP_INTERVAL);

		static bool now(struct FullTime *const fulltime) {
			if (!NTP.isTimeSet()) return false;
			time_t const epoch = NTP.getEpochTime();
			struct tm time;
			gmtime_r(&epoch, &time);
			*fulltime = {
				.year = (unsigned short int)(1900U + time.tm_year),
				.month = (unsigned char)(time.tm_mon + 1),
				.day = (unsigned char)time.tm_mday,
				.hour = (unsigned char)time.tm_hour,
				.minute = (unsigned char)time.tm_min,
				.second = (unsigned char)time.tm_sec
			};
			return true;
		}

		static void synchronize(void) {
			if (NTP.update()) {
				time_t const epoch = NTP.getEpochTime();
				struct tm time;
				gmtime_r(&epoch, &time);
				struct FullTime const fulltime = {
					.year = (unsigned short int)(1900U + time.tm_year),
					.month = (unsigned char)(time.tm_mon + 1),
					.day = (unsigned char)time.tm_mday,
					.hour = (unsigned char)time.tm_hour,
					.minute = (unsigned char)time.tm_min,
					.second = (unsigned char)time.tm_sec
				};
				RTC::set(fulltime);
				COM::println("NTP update");
			}
		}
	}
#endif

/* ************************************************************************** */

namespace Sleep {
	class Unsleep {
	protected:
		bool flag;
	public:
		Unsleep(void);
		virtual bool awake(void) const;
		void set_awake(bool value);
	};

	inline Unsleep::Unsleep(void) : flag(false) {}

	bool Unsleep::awake(void) const {
		return flag;
	}

	void Unsleep::set_awake(bool const value) {
		flag = value;
	}

	#if defined(ENABLE_SLEEP)
		static Time const MAXIMUM_SLEEP_LENGTH = 24 * 60 * 60 * 1000; /* milliseconds */
		static bool enabled = false;
		static Time wake_time = 0;
		static std::vector<class Unsleep const *> unsleep_list;

		inline static bool in_range(Time const period) {
			return 0 < period && period < MAXIMUM_SLEEP_LENGTH;
		}

		static void add_unsleep(class Unsleep const *const u) {
			unsleep_list.push_back(u);
		}

		static void alarm(Time const wake) {
			if (enabled) {
				Time const now = millis();
				Time const period_0 = wake_time - now + SLEEP_MARGIN;
				Time const period_1 = wake - now + SLEEP_MARGIN;
				if (!in_range(period_0)) return;
				if (!in_range(period_1) || period_0 <= period_1) return;
			}
			enabled = true;
			wake_time = wake;
		}

		static void sleep(void) {
			if (!enabled) return;
			for (class Unsleep const *const u: unsleep_list)
				if (u->awake())
					return;
			Time const now = millis();
			Time const milliseconds = wake_time - now - SLEEP_MARGIN;
			if (milliseconds < MAXIMUM_SLEEP_LENGTH) {
				Debug::print("DEBUG: sleep ");
				Debug::print(milliseconds);
				Debug::println("ms");
				Debug::flush();
				LoRa.sleep();
				esp_sleep_enable_timer_wakeup(milliseconds * 1000);
				esp_light_sleep_start();
			}
			enabled = false;
		}
	#else
		static void add_unsleep([[maybe_unused]] class Unsleep const *const u) {}
		inline static void alarm([[maybe_unused]] Time const wake) {}
		inline static void sleep(void) {}
	#endif
}

class Schedule {
protected:
	bool enable;
	Time head;
	Time period;
	Time margin;
public:
	Schedule(Time initial_period);
	bool enabled(void) const;
	Time next_run(Time now) const;
	void start(Time now, Time addition_period = 0);
	void stop(void);
	virtual bool tick(Time now);
	virtual void run(Time now);
};

inline Schedule::Schedule(Time const initial_period) :
	enable(false), head(0), period(initial_period), margin(0)
{}

inline bool Schedule::enabled(void) const {
	return enable;
}

inline Time Schedule::next_run(Time const now) const {
	return head + period + margin;
}

inline void Schedule::start(Time const now, Time const addition) {
	enable = true;
	head = now;
	margin = addition;
}

inline void Schedule::stop(void) {
	enable = false;
}

bool Schedule::tick(Time const now) {
	bool const need_run = enable && now - head >= period + margin;
	if (need_run) run(now);
	if (enable) Sleep::alarm(next_run(now));
	return need_run;
}

void Schedule::run(Time const now) {
	head = now;
}

namespace Schedules {
	static class std::vector<class Schedule *> list;

	inline static void add(class Schedule *const schedule) {
		/* Add [schedule] into the list */
		list.push_back(schedule);
	}

	static void remove(class Schedule *const schedule) {
		/* Remove [schedule] from the list */
		size_t const N = list.size();
		for (size_t i = 0; i < N; ++i) {
			class Schedule *const p = list[i];
			if (p == schedule) {
				list[i] = list.back();
				list.pop_back();
				break;
			}
		}
	}

	static void tick(void) {
		/* Run a schedule on time */
		Time const now = millis();
		for (class Schedule *const schedule: list)
			if (schedule->tick(now))
				return;
		/* Sleep if no schedule to run */
		Sleep::sleep();
	}
}

/* ************************************************************************** */

struct [[gnu::packed]] Data {
	struct FullTime time;
	#ifdef ENABLE_BATTERY_GAUGE
		float battery_voltage;
		float battery_percentage;
	#endif
	#ifdef ENABLE_DALLAS
		float dallas_temperature;
	#endif
	#ifdef ENABLE_SHT40
		float sht40_temperature;
		float sht40_humidity;
	#endif
	#ifdef ENABLE_BME280
		float bme280_temperature;
		float bme280_pressure;
		float bme280_humidity;
	#endif
	#ifdef ENABLE_LTR390
		float ltr390_ultraviolet;
	#endif

	void writeln(class Print *print) const;
	bool readln(class Stream *stream);
};

void Data::writeln(class Print *const print) const {
	print->printf(
		"%04u-%02u-%02uT%02u:%02u:%02uZ,",
		this->time.year, this->time.month, this->time.day,
		this->time.hour, this->time.minute, this->time.second
	);
	#ifdef ENABLE_BATTERY_GAUGE
		print->printf(
			"%f,%f,",
			this->battery_voltage, this->battery_percentage
		);
	#endif
	#ifdef ENABLE_DALLAS
		print->printf(
			"%f,",
			this->dallas_temperature
		);
	#endif
	#ifdef ENABLE_SHT40
		print->printf(
			"%f,%f,",
			this->sht40_temperature, this->sht40_humidity
		);
	#endif
	#ifdef ENABLE_BME280
		print->printf(
			"%f,%f,%f,",
			this->bme280_temperature, this->bme280_pressure, this->bme280_humidity
		);
	#endif
	#ifdef ENABLE_LTR390
		print->printf(
			"%f,",
			this->ltr390_ultraviolet
		);
	#endif
	print->write('\n');
}

bool Data::readln(class Stream *const stream) {
	/* Time */
	{
		class String const s = stream->readStringUntil(',');
		if (
			sscanf(
				s.c_str(),
				"%4hu-%2hhu-%2hhuT%2hhu:%2hhu:%2hhuZ",
				&this->time.year, &this->time.month, &this->time.day,
				&this->time.hour, &this->time.minute, &this->time.second
			) != 6
		) return false;
	}

	/* Battery gauge */
	#ifdef ENABLE_BATTERY_GAUGE
		{
			class String const s = stream->readStringUntil(',');
			if (sscanf(s.c_str(), "%f", &this->battery_voltage) != 1) return false;
		}
		{
			class String const s = stream->readStringUntil(',');
			if (sscanf(s.c_str(), "%f", &this->battery_percentage) != 1) return false;
		}
	#endif

	/* Dallas thermometer */
	#ifdef ENABLE_DALLAS
		{
			class String const s = stream->readStringUntil(',');
			if (sscanf(s.c_str(), "%f", &this->dallas_temperature) != 1) return false;
		}
	#endif

	/* Dallas thermometer */
	#ifdef ENABLE_SHT40
		{
			class String const s = stream->readStringUntil(',');
			if (sscanf(s.c_str(), "%f", &this->sht40_temperature) != 1) return false;
		}
		{
			class String const s = stream->readStringUntil(',');
			if (sscanf(s.c_str(), "%f", &this->sht40_humidity) != 1) return false;
		}
	#endif

	/* BME280 sensor */
	#ifdef ENABLE_BME280
		{
			class String const s = stream->readStringUntil(',');
			if (sscanf(s.c_str(), "%f", &this->bme280_temperature) != 1) return false;
		}
		{
			class String const s = stream->readStringUntil(',');
			if (sscanf(s.c_str(), "%f", &this->bme280_pressure) != 1) return false;
		}
		{
			class String const s = stream->readStringUntil(',');
			if (sscanf(s.c_str(), "%f", &this->bme280_humidity) != 1) return false;
		}
	#endif

	/* LTR390 sensor */
	#ifdef ENABLE_LTR390
		{
			class String const s = stream->readStringUntil(',');
			if (sscanf(s.c_str(), "%f", &this->ltr390_ultraviolet) != 1) return false;
		}
	#endif

	stream->readStringUntil('\n');
	return true;
}

/* ************************************************************************** */

#if defined(ENABLE_GATEWAY)
	#include <HTTPClient.h>

	namespace WIFI {
		static void initialize(void) {
			WiFi.begin(WIFI_SSID, WIFI_PASS);
		}

		static class String status_message(signed int const WiFi_status) {
			switch (WiFi_status) {
			case WL_NO_SHIELD:
				return String("WiFi no shield");
			case WL_IDLE_STATUS:
				return String("WiFi idle");
			case WL_NO_SSID_AVAIL:
				return String("WiFi no SSID");
			case WL_SCAN_COMPLETED:
				return String("WiFi scan completed");
			case WL_CONNECTED:
				return String("WiFi connected");
			case WL_CONNECT_FAILED:
				return String("WiFi connect failed");
			case WL_CONNECTION_LOST:
				return String("WiFi connection lost");
			case WL_DISCONNECTED:
				return String("WiFi disconnected");
			default:
				return String("WiFi Status: ") + String(WiFi_status);
			}
		}

		static bool upload(Device const device, SerialNumber const serial, struct Data const *const data) {
			signed int const WiFi_status = WiFi.status();
			if (WiFi_status != WL_CONNECTED) {
				any_print("No WiFi: ");
				any_println(status_message(WiFi.status()));
				return false;
			}
			class String const time = String(data->time);
			class HTTPClient HTTP_client;
			char URL[HTTP_UPLOAD_LENGTH];
			snprintf(
				URL, sizeof URL,
				HTTP_UPLOAD_FORMAT,
				device, serial, time.c_str()
				#ifdef ENABLE_BATTERY_GAUGE
					, data->battery_voltage
					, data->battery_percentage
				#endif
				#ifdef ENABLE_DALLAS
					, data->dallas_temperature
				#endif
				#ifdef ENABLE_SHT40
					, data->sht40_temperature
					, data->sht40_humidity
				#endif
				#ifdef ENABLE_BME280
					, data->bme280_temperature
					, data->bme280_pressure
					, data->bme280_humidity
				#endif
				#ifdef ENABLE_LTR390
					, data->ltr390_ultraviolet
				#endif
			);
			COM::print("Upload to ");
			COM::println(URL);
			HTTP_client.begin(URL);
			static char const authorization_type[] = HTTP_AUTHORIZATION_TYPE;
			static char const authorization_code[] = HTTP_AUTHORIZATION_CODE;
			if (authorization_type[0] && authorization_code[0]) {
				HTTP_client.setAuthorizationType(authorization_type);
				HTTP_client.setAuthorization(authorization_code);
			}
			signed int HTTP_status = HTTP_client.GET();
			any_print("HTTP status: ");
			any_println(HTTP_status);
			if (not (HTTP_status >= 200 and HTTP_status < 300)) return false;
			return true;
		}

		static void loop(void) {
			static wl_status_t last_WiFi = WL_IDLE_STATUS;
			wl_status_t this_WiFi = WiFi.status();
			if (this_WiFi != last_WiFi) {
				COM::print("WiFi status: ");
				COM::println(status_message(WiFi.status()));
				last_WiFi = this_WiFi;
			}
			if (this_WiFi == WL_CONNECTED) {
				NTP::synchronize();
			}
		}
	}
#else
	namespace WIFI {
		inline static void initialize(void) {
			/* stop WiFi to lower power consumption */
			WiFi.mode(WIFI_OFF);
		}
		inline static void loop(void) {}
	}
#endif

/* ************************************************************************** */

namespace LORA {
	static Device last_receiver = 0;

	namespace Send {
		static bool payload(char const *const message, void const *const content, size_t const size) {
			Debug::print("DEBUG: LORA::Send::payload ");
			Debug::println(message);

			uint8_t nonce[CIPHER_IV_LENGTH];
			RNG.rand(nonce, sizeof nonce);
			AuthCipher cipher;
			if (!cipher.setKey((uint8_t const *)secret_key, sizeof secret_key)) {
				COM::print("LoRa ");
				COM::print(message);
				COM::println(": unable to set key");
				#if defined(ENABLE_OLED_OUTPUT)
					OLED::set_message("Unable to set key\n");
				#endif
				return false;
			}
			if (!cipher.setIV(nonce, sizeof nonce)) {
				COM::print("LoRa ");
				COM::print(message);
				COM::println(": unable to set nonce");
				#if defined(ENABLE_OLED_OUTPUT)
					OLED::set_message("Unable to set nonce\n");
				#endif
				return false;
			}
			uint8_t ciphertext[size];
			cipher.encrypt(ciphertext, (uint8_t const *)content, size);
			uint8_t tag[CIPHER_TAG_SIZE];
			cipher.computeTag(tag, sizeof tag);
			LoRa.write(nonce, sizeof nonce);
			LoRa.write(ciphertext, sizeof ciphertext);
			LoRa.write(tag, sizeof tag);

			Debug::dump("DEBUG: LoRa::Send::payload", content, size);
			return true;
		}

		static void ASKTIME(void) {
			LoRa.beginPacket();
			LoRa.write(uint8_t(PACKET_ASKTIME));
			LoRa.write(uint8_t(last_receiver));
			Device const device = DEVICE_ID;
			LORA::Send::payload("ASKTIME", &device, sizeof device);
			LoRa.endPacket();
		}

		static void SEND(Device const receiver, SerialNumber const serial, Data const *const data) {
			Debug::print("DEBUG: LoRa::Send::SEND receiver=");
			Debug::print(receiver);
			Debug::print(" serial=");
			Debug::println(serial);

			Device const device = DEVICE_ID;
			unsigned char payload[2 * sizeof device + sizeof serial + sizeof *data];
			std::memcpy(payload, &device, sizeof device);
			std::memcpy(payload + sizeof device, &device, sizeof device);
			std::memcpy(payload + 2 * sizeof device, &serial, sizeof serial);
			std::memcpy(payload + 2 * sizeof device + sizeof serial, data, sizeof *data);
			LoRa.beginPacket();
			LoRa.write(uint8_t(PACKET_SEND));
			LoRa.write(uint8_t(receiver));
			LORA::Send::payload("SEND", payload, sizeof payload);
			LoRa.endPacket();
		}
	}
}

/* ************************************************************************** */

#if defined(ENABLE_GATEWAY)
	static class Synchronize : public Schedule {
	public:
		Synchronize(void);
		virtual void run(Time now);
		static void initialize(void);
	} synchronize_schedule;

	inline Synchronize::Synchronize(void) : Schedule(SYNCHONIZE_INTERVAL) {}

	void Synchronize::run(Time const now) {
		Debug::print("Synchronize::run ");
		Debug::println(now);
		Schedule::run(now);

		struct FullTime fulltime;
		if (!NTP::now(&fulltime)) return;
		RTC::set(fulltime);

		LoRa.beginPacket();
		LoRa.write(PacketType(PACKET_TIME));
		LoRa.write(Device(0));
		LORA::Send::payload("TIME", &fulltime, sizeof fulltime);
		LoRa.endPacket();

		OLED::home();
		any_print("Synchronize: ");
		any_println(String(fulltime));
		OLED::display();
	}

	void Synchronize::initialize(void) {
		Schedules::add(&synchronize_schedule);
	}
#else
	namespace Synchronize {
		inline static void initialize(void) {}
	}
#endif

/* ************************************************************************** */

#if defined(ENABLE_SLEEP)
	class AskTime : public Schedule {
	protected:
		bool wait_response;
		class Sleep::Unsleep unsleep;
	public:
		AskTime(void);
		void start(Time const now);
		virtual void run(Time now) override;
		void reset(void);
		static void initialize(void);
	} ask_time_schedule;

	inline AskTime::AskTime(void) :
		Schedule(SYNCHONIZE_INTERVAL), wait_response(false), unsleep()
	{}

	void AskTime::start(Time const now) {
		Debug::print("AskTime::start ");
		Debug::print(now);
		Debug::print(", period = ");
		Debug::println(period);
		Schedule::start(now - period, rand_int<uint8_t>());
	}

	void AskTime::run(Time const now) {
		Debug::print("Asktime::run ");
		Debug::println(now);
		Schedule::run(now);
		if (!wait_response) {
			wait_response = true;
			period = SYNCHONIZE_MARGIN;
			unsleep.set_awake(true);
			LORA::Send::ASKTIME();
		} else {
			reset();
		}
	}

	void AskTime::reset(void) {
		wait_response = false;
		period = SYNCHONIZE_INTERVAL;
		unsleep.set_awake(false);
	}

	void AskTime::initialize(void) {
		Debug::println("AskTime::initialize");
		Schedules::add(&ask_time_schedule);
		Sleep::add_unsleep(&ask_time_schedule.unsleep);
		ask_time_schedule.start(0);
	}
#else
	namespace AskTime {
		inline static void initialize(void) {}
	}
#endif

/* ************************************************************************** */

#if defined(ENABLE_MEASURE)
	static SerialNumber current_serial;

	#if defined(ENABLE_OLED_OUTPUT)
		static void draw_received(void) {
			OLED::SSD1306.drawRect(125, 61, 3, 3, SSD1306_WHITE);
			OLED::SSD1306.display();
		}
	#else
		inline static void draw_received(void) {}
	#endif

	#if defined(ENABLE_SD_CARD)
		static bool lock_push = false;
	#endif

	class Sender : public Schedule {
	protected:
		unsigned int retry;
		Device receiver;
		SerialNumber serial;
		struct Data data;
		Device next_router(void);
		class Sleep::Unsleep unsleep;
	public:
		Sender(void);
		void start(Time const now);
		virtual void run(Time now) override;
		void start_send(struct Data const *const data);
		bool stop_ack(SerialNumber const serial);
		static void initialize(void);
	} sender_schedule;

	Sender::Sender(void) : Schedule(SEND_INTERVAL), unsleep() {
	}

	Device Sender::next_router(void) {
		/* Return next router or DEVICE_ID if no more routers */
		size_t const N = sizeof router_topology / sizeof *router_topology;
		size_t i = 0;
		for (;;) {
			if (i >= N) return DEVICE_ID;
			if (router_topology[i][1] == DEVICE_ID && router_topology[i][0] == receiver) break;
			++i;
		}
		size_t j = i + 1;
		for (;;) {
			if (j >= N) j = 0;
			if (router_topology[j][1] == DEVICE_ID) {
				Device const device = router_topology[j][0];
				if (device == LORA::last_receiver) return DEVICE_ID;
				return device;
			}
			++j;
		}
	}

	void Sender::start(Time const now) {
		if (!RESEND_TIMES) return;
		Schedule::start(now, rand_int<uint8_t>());
	}

	void Sender::run(Time const now) {
		Schedule::run(now);
		if (retry) {
			--retry;
			LORA::Send::SEND(receiver, serial, &data);
		} else {
			Device const next = next_router();
			if (next == DEVICE_ID) {
				stop();
				#if defined(ENABLE_SD_CARD)
					lock_push = true;
				#endif
			} else {
				receiver = next;
				retry = RESEND_TIMES;
			}
		}
	}

	void Sender::start_send(struct Data const *const data) {
		retry = RESEND_TIMES;
		receiver = LORA::last_receiver;
		serial = current_serial;
		++current_serial;
		this->data = *data;
		unsleep.set_awake(true);
		Time const now = millis();
		start(now);
		run(now);
	}

	bool Sender::stop_ack(SerialNumber const ack_serial) {
		if (ack_serial == serial) {
			unsleep.set_awake(false);
			LORA::last_receiver = receiver;
			stop();
			return true;
		} else {
			COM::println("LoRa ACK: serial number unmatched");
			return false;
		}
	}

	void Sender::initialize(void) {
		Schedules::add(&sender_schedule);
		Sleep::add_unsleep(&sender_schedule.unsleep);
	}

	bool send_data(struct Data const *const data) {
		#if defined(ENABLE_GATEWAY)
			if (WIFI::upload(DEVICE_ID, current_serial++, data)) {
				draw_received();
				return true;
			} else {
				COM::print("HTTP: unable to send data: time=");
				COM::println(String(data->time));
				return false;
			}
		#else
			sender_schedule.start_send(data);
			return false;
		#endif
	}

	#if defined(ENABLE_SD_CARD)
		#include <SD.h>

		namespace SD_CARD {
			static class SPIClass SPI_1(HSPI);

			static void cleanup(void) {
				if (SD.exists(cleanup_file_path))
					SD.remove(data_file_path);
				else if (!SD.rename(data_file_path, cleanup_file_path))
					return;
				if (!SD.exists(cleanup_file_path)) return;
				class File cleanup_file = SD.open(cleanup_file_path, "r");
				if (!cleanup_file) {
					COM::println("Fail to open clean-up file");
					return;
				}
				class File data_file = SD.open(data_file_path, "w");
				if (!data_file) {
					COM::println("Fail to create data file");
					cleanup_file.close();
					return;
				}

				#if !defined(DEBUG_CLEAN_OLD_DATA)
					for (;;) {
						class String const s = cleanup_file.readStringUntil(',');
						if (!s.length()) break;
						bool const sent = s != "0";

						struct Data data;
						if (!data.readln(&cleanup_file)) {
							COM::println("Clean-up: invalid data");
							break;
						}

						if (!sent) {
							data_file.print("0,");
							data.writeln(&data_file);
						}
					}
				#endif

				cleanup_file.close();
				data_file.close();
				SD.remove(cleanup_file_path);
			}

			static void append(struct Data const *const data) {
				#if defined(ENABLE_LOG_FILE)
					class File log_file = SD.open(log_file_path, "a");
					if (!log_file) {
						any_println("Cannot open log file");
					}
					else {
						try {
							data->writeln(&log_file);
						} catch (...) {
							any_println("Cannot append log file");
						}
						log_file.close();
					}
				#endif

				class File data_file = SD.open(data_file_path, "a");
				if (!data_file) {
					any_println("Cannot append data file");
				} else {
					try {
						data_file.print("0,");
						data->writeln(&data_file);
					} catch (...) {
						any_println("Cannot append data file");
					}
					data_file.close();
				}
			}

			static class Push : public Schedule {
			protected:
				off_t current_position;
				off_t next_position;
			public:
				Push(void);
				virtual void run(Time now);
				void ack(void);
			} push_schedule;

			inline Push::Push(void) :
				Schedule(SEND_INTERVAL), current_position(0), next_position(0)
			{}

			void Push::run(Time const now) {
				Debug::print("Push::run ");
				Debug::println(now);
				if (lock_push) return;
				Schedule::run(now);
				class File data_file = SD.open(DATA_FILE_PATH, "r+", true);
				if (!data_file) {
					COM::println("Push: fail to open data file");
					return;
				}
				if (!data_file.seek(current_position)) {
					COM::print("Push: cannot seek to ");
					COM::println(current_position);
					data_file.close();
					return;
				}
				for (;;) {
					class String const s = data_file.readStringUntil(',');
					if (!s.length()) {
						stop();
						break;
					}
					bool const sent = s != "0";
					struct Data data;
					if (!data.readln(&data_file)) {
						COM::print("Push: invalid data at ");
						COM::println(data_file.position());
						break;
					}
					next_position = data_file.position();
					if (sent) {
						current_position = next_position;
					} else {
						if (send_data(&data)) {
							if (!data_file.seek(current_position)) {
								COM::print("Push: fail to seek data file: ");
								COM::println(current_position);
							} else {
								data_file.write('1');
								current_position = next_position;
							}
						}
						break;
					}
				}
				data_file.close();
			}

			void Push::ack(void) {
				Debug::println("Push::ack");
				if (current_position == next_position) return;
				class File data_file = SD.open(data_file_path, "r+");
				if (!data_file) {
					COM::println("LoRa ACK: fail to open data file");
					return;
				}
				if (!data_file.seek(current_position)) {
					COM::print("LoRa ACK: fail to seek data file: ");
					COM::println(current_position);
					data_file.close();
					return;
				}
				data_file.write('1');
				data_file.close();

				current_position = next_position;

				this->start(millis());
			}

			#if defined(CLEANLOG_INTERVAL) && CLEANLOG_INTERVAL > 0
				static class CleanLog : public Schedule {
				public:
					CleanLog(void);
					virtual void run(Time now) override;
				} cleanlog_schedule;

				CleanLog::CleanLog(void) : Schedule(CLEANLOG_INTERVAL) {}

				void CleanLog::run(Time const now) {
					Schedule::run(now);
					cleanup();
				}
			#endif

			static bool initialize(void) {
				pinMode(SD_MISO, INPUT_PULLUP);
				SPI_1.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
				if (SD.begin(SD_CS, SPI_1)) {
					any_println("SD card initialized");
					COM::println(String("SD Card type: ") + String(SD.cardType()));
					any_println("Cleaning up data file");
					OLED::display();
					cleanup();
					any_println("Data file cleaned");
					OLED::display();
					Schedules::add(&push_schedule);
					Schedules::add(&cleanlog_schedule);
					return true;
				} else {
					any_println("SD card uninitialized");
					OLED::display();
					return false;
				}
			}
		}
	#endif

	#if defined(ENABLE_BATTERY_GAUGE)
		#if ENABLE_BATTERY_GAUGE == BATTERY_GAUGE_DFROBOT
			#include <DFRobot_MAX17043.h>

			static class DFRobot_MAX17043 battery;
		#elif ENABLE_BATTERY_GAUGE == BATTERY_GAUGE_LC709203F
			#include <Adafruit_LC709203F.h>

			static class Adafruit_LC709203F battery;
		#endif
	#endif

	#ifdef ENABLE_DALLAS
		#include <OneWire.h>
		#include <DallasTemperature.h>

		static class OneWire onewire_thermometer(ENABLE_DALLAS);
		static class DallasTemperature dallas(&onewire_thermometer);
	#endif

	#ifdef ENABLE_SHT40
		#include <Adafruit_SHT4x.h>

		static class Adafruit_SHT4x SHT = Adafruit_SHT4x();
	#endif

	#ifdef ENABLE_BME280
		#include <Adafruit_Sensor.h>
		#include <Adafruit_BME280.h>

		static class Adafruit_BME280 BME;
	#endif

	#ifdef ENABLE_LTR390
		#include <Adafruit_LTR390.h>
		static class Adafruit_LTR390 LTR;
	#endif

	static class Measure : public Schedule {
	public:
		Measure(void);
		virtual void run(Time now) override;
		void measured(Time now, struct Data const *const data);
		static bool initialize(void);
	} measure_schedule;

	inline Measure::Measure(void) : Schedule(MEASURE_INTERVAL) {}

	void Measure::run(Time const now) {
		Debug::print("Measure::run ");
		Debug::println(now);
		Schedule::run(now);

		struct Data data;
		if (!RTC::now(&data.time)) return;
		OLED::home();
		any_print("Mesaure device ");
		any_println(DEVICE_ID);
		COM::print("Time: ");
		any_println(String(data.time));

		#if defined(ENABLE_BATTERY_GAUGE)
			#if ENABLE_BATTERY_GAUGE == BATTERY_GAUGE_DFROBOT
				data.battery_voltage = battery.readVoltage() / 1000;
				data.battery_percentage = battery.readPercentage();
			#elif ENABLE_BATTERY_GAUGE == BATTERY_GAUGE_LC709203F
				data.battery_voltage = battery.cellVoltage();
				data.battery_percentage = battery.cellPercent();
			#endif
			any_print("Bat: ");
			any_print(data.battery_voltage);
			any_print("V ");
			any_print(data.battery_percentage);
			any_println("%");
		#endif

		#if defined(ENABLE_DALLAS)
			data.dallas_temperature = dallas.getTempCByIndex(0);
			any_print("Dallas temp.: ");
			any_println(data.dallas_temperature);
		#endif

		#if defined(ENABLE_SHT40)
			{
				sensors_event_t temperature_event, humidity_event;
				SHT.getEvent(&humidity_event, &temperature_event);
				data.sht40_temperature = temperature_event.temperature;
				data.sht40_humidity = humidity_event.relative_humidity;
				any_print("SHT temp.: ");
				any_println(data.sht40_temperature);
				any_print("SHT humidity: ");
				any_println(data.sht40_humidity);
			}
		#endif

		#if defined(ENABLE_BME280)
			data.bme280_temperature = BME.readTemperature();
			data.bme280_pressure = BME.readPressure();
			data.bme280_humidity = BME.readHumidity();
			any_print("BME temp.: ");
			any_println(data.bme280_temperature);
			any_print("BME pressure: ");
			any_println(data.bme280_pressure, 0);
			any_print("BME humidity: ");
			any_println(data.bme280_humidity);
		#endif

		#if defined(ENABLE_LTR390)
			data.ltr390_ultraviolet = LTR.readUVS();
			any_print("LTR UV: ");
			any_println(data.ltr390_ultraviolet);
		#endif

		#if defined(ENABLE_OLED_OUTPUT)
			OLED::print_message();
			OLED::set_message("");
		#endif

		measured(now, &data);
		OLED::display();
	}

	#if defined(ENABLE_SD_CARD)
		void Measure::measured(Time const now, struct Data const *const data) {
			SD_CARD::append(data);
			lock_push = false;
			SD_CARD::push_schedule.start(now);
		}
	#else
		void Measure::measured([[maybe_unused]] Time const now, struct Data const *const data) {
			send_data(data);
		}
	#endif

	bool Measure::initialize(void) {
		/* Initial battery gauge */
		#if defined(ENABLE_BATTERY_GAUGE)
			battery.begin();
		#endif

		/* Initialize Dallas thermometer */
		#if defined(ENABLE_DALLAS)
			dallas.begin();
			DeviceAddress thermometer_address;
			if (dallas.getAddress(thermometer_address, 0)) {
				any_println("Thermometer 0 found");
			} else {
				any_println("Thermometer 0 not found");
				return false;
			}
		#endif

		/* Initialize SHT40 sensor */
		#if defined(ENABLE_SHT40)
			if (SHT.begin()) {
				any_println("SHT40 sensor found");
			} else {
				any_println("SHT40 sensor not found");
				return false;
			}
			SHT.setPrecision(SHT4X_HIGH_PRECISION);
			SHT.setHeater(SHT4X_NO_HEATER);
		#endif

		/* Initialize BME280 sensor */
		#if defined(ENABLE_BME280)
			if (BME.begin()) {
				any_println("BME280 sensor found");
			} else {
				any_println("BME280 sensor not found");
				return false;
			}
		#endif

		/* Initial LTR390 sensor */
		#if defined(ENABLE_LTR390)
			if (LTR.begin()) {
				LTR.setMode(LTR390_MODE_UVS);
				any_println("LTR390 sensor found");
			} else {
				any_println("LTR390 sensor not found");
				return false;
			}
		#endif

		/* Schedule to measure */
		Schedules::add(&measure_schedule);
		measure_schedule.start(0);
		measure_schedule.run(0);

		return true;
	}
#endif

#if !defined(ENABLE_MEASURE) || !defined(ENABLE_SD_CARD)
	namespace SD_CARD {
		inline static bool initialize(void) {
			return true;
		}
	}
#endif

#if !defined(ENABLE_MEASURE)
	namespace Sender {
		inline static void initialize(void) {
		}
	}

	namespace Measure {
		inline static bool initialize(void) {
			return true;
		}
	}
#endif

/* ************************************************************************** */

namespace LORA {
	namespace Receive {
		static bool payload(char const *const message, void *const content, size_t const size) {
			uint8_t nonce[CIPHER_IV_LENGTH];
			if (LoRa.readBytes(nonce, sizeof nonce) != sizeof nonce) {
				COM::print("LoRa ");
				COM::print(message);
				COM::println(": fail to read cipher nonce");
				#ifdef ENABLE_OLED_OUTPUT
					OLED::set_message(String("LoRa ") + message + ": fail to read cipher nonce\n");
				#endif
				return false;
			}
			char ciphertext[size];
			if (LoRa.readBytes(ciphertext, sizeof ciphertext) != sizeof ciphertext) {
				COM::print("LoRa ");
				COM::print(message);
				COM::println(": fail to read time");
				#ifdef ENABLE_OLED_OUTPUT
					OLED::set_message(String("LoRa ") + message + ": fail to read time\n");
				#endif
				return false;
			}
			uint8_t tag[CIPHER_TAG_SIZE];
			if (LoRa.readBytes(tag, sizeof tag) != sizeof tag) {
				COM::print("LoRa ");
				COM::print(message);
				COM::println(": fail to read cipher tag");
				#ifdef ENABLE_OLED_OUTPUT
					OLED::set_message(String("LoRa ") + message + ": fail to read cipher tag\n");
				#endif
				return false;
			}
			AuthCipher cipher;
			if (!cipher.setKey((uint8_t const *)secret_key, sizeof secret_key)) {
				COM::print("LoRa ");
				COM::print(message);
				COM::println(": fail to set cipher key");
				#ifdef ENABLE_OLED_OUTPUT
					OLED::set_message(String("LoRa ") + message + ": fail to set cipher key\n");
				#endif
				return false;
			}
			if (!cipher.setIV(nonce, sizeof nonce)) {
				COM::print("LoRa ");
				COM::print(message);
				COM::println(": fail to set cipher nonce");
				#ifdef ENABLE_OLED_OUTPUT
					OLED::set_message(String("LoRa ") + message + ": fail to set cipher nonce\n");
				#endif
				return false;
			}
			cipher.decrypt((uint8_t *)content, (uint8_t const *)&ciphertext, size);
			if (!cipher.checkTag(tag, sizeof tag)) {
				COM::print("LoRa ");
				COM::print(message);
				COM::println(": invalid cipher tag");
				#ifdef ENABLE_OLED_OUTPUT
					OLED::set_message(String("LoRa ") + message + ": invalid cipher tag\n");
				#endif
				return false;
			}

			Debug::dump("DEBUG: LoRa::Receive::payload", content, size);
			return true;
		}

		#if defined(ENABLE_GATEWAY)
			static Time last_time = 0;
			static SerialNumber last_serial[NUMBER_OF_DEVICES];

			inline static void TIME([[maybe_unused]] signed int const packet_size) {}

			static void ASKTIME(signed int const packet_size) {
				size_t const overhead_size =
					sizeof (PacketType) /* packet type */
					+ sizeof (Device)   /* receiver */
					+ CIPHER_IV_LENGTH  /* nonce */
					+ CIPHER_TAG_SIZE;  /* cipher tag */
				size_t const expected_packet_size =
					overhead_size
					+ sizeof (Device);  /* terminal */
				if (packet_size != expected_packet_size) {
					COM::print("LoRa ASKTIME: incorrect packet size: ");
					COM::println(packet_size);
					return;
				}

				Device receiver;
				if (LoRa.readBytes(&receiver, sizeof receiver) != sizeof receiver) return;
				if (receiver != (Device)0) return;

				Device device;
				if (!payload("ASKTIME", &device, sizeof device)) return;
				if (!(device > 0 && device < NUMBER_OF_DEVICES)) {
					COM::print("LoRa ASKTIME: incorrect device: ");
					COM::println(device);
					return;
				}

				Debug::print("DEBUG: LoRa::Receive::ASKTIME ");
				Debug::println(device);

				synchronize_schedule.run(millis());
			}

			static void SEND(signed int const packet_size) {
				Debug::println("DEBUG: LoRa::Receive::SEND");
				size_t const overhead_size =
					sizeof (PacketType)     /* packet type */
					+ sizeof (Device)       /* receiver */
					+ CIPHER_IV_LENGTH      /* nonce */
					+ CIPHER_TAG_SIZE;      /* cipher tag */
				size_t const minimal_packet_size =
					overhead_size
					+ sizeof (Device)       /* terminal */
					+ sizeof (Device)       /* router list length >= 1 */
					+ sizeof (SerialNumber) /* serial code */
					+ sizeof (struct Data); /* data */
				if (!(packet_size >= minimal_packet_size)) {
					COM::print("LoRa SEND: incorrect packet size: ");
					COM::println(packet_size);
					return;
				}
				Device receiver;
				if (LoRa.readBytes(&receiver, sizeof receiver) != sizeof receiver) return;
				if (receiver != (Device)0) return;
				size_t const payload_size = packet_size - overhead_size;
				unsigned char content[payload_size];
				if (!LORA::Receive::payload("SEND", &content, sizeof content)) return;

				Device device;
				std::memcpy(&device, content, sizeof device);
				if (!(device > 0 && device < NUMBER_OF_DEVICES)) {
					COM::print("LoRa SEND: incorrect device: ");
					COM::println(device);
					return;
				}

				size_t routers_length = sizeof (Device);
				for (;;) {
					if (routers_length >= payload_size) {
						COM::println("LoRa SEND: incorrect router list");
						return;
					}
					Device router;
					std::memcpy(&router, content + routers_length, sizeof router);
					if (router == device) break;
					routers_length += sizeof router;
				}
				size_t const excat_packet_size =
					minimal_packet_size
					+ routers_length * sizeof (Device)
					- sizeof (Device);
				if (packet_size != excat_packet_size) {
					COM::print("LoRa SEND: incorrect packet size or router list: ");
					COM::print(packet_size);
					COM::print(" / ");
					COM::println(routers_length);
					return;
				}

				SerialNumber serial;
				std::memcpy(&serial, content + sizeof (Device) + routers_length, sizeof serial);
				if (
					!(serial >= LORA::Receive::last_serial[device])
					&& !(LORA::Receive::last_serial[device] & ~(~(SerialNumber)0 >> 1))
				)
					COM::println("LoRa SEND: serial number out of order");

				size_t const router_list_size =
					sizeof (Device) * (1 + routers_length)
					+ sizeof (SerialNumber);
				struct Data data;
				std::memcpy(&data, content + router_list_size, sizeof data);

				LORA::Receive::last_serial[device] = serial;
				#ifdef ENABLE_OLED_OUTPUT
					OLED::home();
					OLED::print("Receive ");
					OLED::print(device);
					OLED::print(" Serial ");
					OLED::println(serial);
					OLED::println(String(data.time));
					#if defined(ENABLE_BATTERY_GAUGE)
						OLED::print("Battery:");
						OLED::print(data.battery_voltage);
						OLED::print("V ");
						OLED::print(data.battery_percentage);
						OLED::println('%');
					#endif
					#if defined(ENABLE_DALLAS)
						OLED::print("Dallas temp.: ");
						OLED::println(data.dallas_temperature);
					#endif
					#if defined(ENABLE_SHT40)
						OLED::print("SHT temp.: ");
						OLED::println(data.sht40_temperature);
						OLED::print("SHT humidity: ");
						OLED::println(data.sht40_humidity);
					#endif
					#if defined(ENABLE_BME280)
						OLED::print("BME temp.: ");
						OLED::println(data.bme280_temperature);
						OLED::print("BME pressure: ");
						OLED::println(data.bme280_pressure, 0);
						OLED::print("BME humidity: ");
						OLED::println(data.bme280_humidity);
					#endif
					#if defined(ENABLE_LTR390)
						OLED::print("LTR UV: ");
						OLED::println(data.ltr390_ultraviolet);
					#endif
					OLED::display();
				#endif

				bool const OK = WIFI::upload(device, serial, &data);
				OLED::display();
				if (!OK) return;

				Device router;
				std::memcpy(&router, content + sizeof device, sizeof router);

				LoRa.beginPacket();
				LoRa.write(PacketType(PACKET_ACK));
				LoRa.write(router);
				LORA::Send::payload("ACK", content, router_list_size);
				LoRa.endPacket();
			}

			static void ACK([[maybe_unused]] signed int const packet_size) {}
		#endif

		#if !defined(ENABLE_GATEWAY)
			static void TIME(signed int const packet_size) {
				signed int const excat_packet_size =
					sizeof (PacketType)        /* packet type */
					+ sizeof (Device)          /* sender */
					+ CIPHER_IV_LENGTH         /* nonce */
					+ sizeof (struct FullTime) /* time */
					+ CIPHER_TAG_SIZE;         /* cipher tag */
				if (packet_size != excat_packet_size) return;
				Device sender;
				if (LoRa.readBytes(&sender, sizeof sender) != sizeof sender) return;
				struct FullTime time;
				if (!LORA::Receive::payload("TIME", &time, sizeof time)) return;

				if (sender != Device(0)) { /* always accept TIME packet from gateway */
					size_t i = 0;
					for (;;) {
						if (i >= sizeof router_topology / sizeof *router_topology) return;
						if (router_topology[i][0] == DEVICE_ID && router_topology[i][1] == sender) break;
						++i;
					}
				}

				#ifdef ENABLE_SLEEP
					ask_time_schedule.reset();
				#endif
				RTC::set(time);

				Debug::print("DEBUG: LoRa::Receive::TIME ");
				Debug::println(String(time));

				LoRa.beginPacket();
				LoRa.write(PacketType(PACKET_TIME));
				LoRa.write(Device(DEVICE_ID));
				LORA::Send::payload("TIME+", &time, sizeof time);
				LoRa.endPacket();
			}

			inline static void ASKTIME([[maybe_unused]] signed int const packet_size) {}

			static void SEND(signed int const packet_size) {
				size_t const overhead_size =
					sizeof (PacketType)     /* packet type */
					+ sizeof (Device)       /* receiver */
					+ CIPHER_IV_LENGTH      /* nonce */
					+ CIPHER_TAG_SIZE;      /* cipher tag */
				size_t const minimal_packet_size =
					sizeof (Device)         /* terminal */
					+ sizeof (Device)       /* router list length >= 1 */
					+ sizeof (SerialNumber) /* serial code */
					+ sizeof (struct Data); /* data */

				if (!(packet_size >= minimal_packet_size)) {
					COM::print("LoRa SEND: incorrect packet size: ");
					COM::println(packet_size);
					return;
				}
				Device receiver;
				if (LoRa.readBytes(&receiver, sizeof receiver) != sizeof receiver) return;
				Debug::print("DEBUG: LoRa::Receive::SEND receiver=");
				Debug::println(receiver);
				if (receiver != Device(DEVICE_ID)) return;
				unsigned char content[sizeof (Device) + packet_size - overhead_size];
				if (!LORA::Receive::payload("SEND", content + 1, sizeof content - 1)) return;

				std::memcpy(content, content + sizeof (Device), sizeof (Device));
				std::memcpy(content + sizeof (Device), &receiver, sizeof (Device));

				Debug::print("DEBUG: LoRa::Receive::SEND last_receiver=");
				Debug::println(last_receiver);

				LoRa.beginPacket();
				LoRa.write(PacketType(PACKET_SEND));
				LoRa.write(last_receiver);
				LORA::Send::payload("SEND+", content, sizeof content);
				LoRa.endPacket();
			}

			static void ACK(signed int const packet_size) {
				size_t const overhead_size =
					sizeof (PacketType)      /* packet type */
					+ sizeof (Device)        /* receiver */
					+ CIPHER_IV_LENGTH       /* nonce */
					+ CIPHER_TAG_SIZE;       /* cipher tag */
				size_t const minimal_packet_size =
					overhead_size +
					+ sizeof (Device)        /* terminal */
					+ sizeof (Device)        /* router list length >= 1 */
					+ sizeof (SerialNumber); /* serial code */
				if (!(packet_size >= minimal_packet_size)) {
					COM::print("LoRa ACK: incorrect packet size: ");
					COM::println(packet_size);
					return;
				}
				Device receiver;
				if (LoRa.readBytes(&receiver, sizeof receiver) != sizeof receiver) return;

				Debug::print("DEBUG: LoRa::Receive::ACK receiver=");
				Debug::println(receiver);

				if (Device(DEVICE_ID) != receiver) return;
				unsigned char content[packet_size - overhead_size];
				if (!payload("ACK", content, sizeof content)) return;

				Device terminal, router0;
				std::memcpy(&terminal, content, sizeof terminal);
				std::memcpy(&router0, content + sizeof terminal, sizeof router0);
				if (Device(DEVICE_ID) == terminal) {
					if (Device(DEVICE_ID) != router0) {
						COM::print("LoRa ACK: dirty router list");
						return;
					}

					SerialNumber serial;
					std::memcpy(&serial, content + 2 * sizeof (Device), sizeof serial);

					Debug::print("DEBUG: LoRa::Receive::ACK serial=");
					Debug::println(serial);

					if (!sender_schedule.stop_ack(serial)) return;

					#ifdef ENABLE_SD_CARD
						SD_CARD::push_schedule.ack();
					#endif

					draw_received();
				} else {
					Device router1;
					std::memcpy(&router1, content + sizeof terminal + sizeof router0, sizeof router1);
					Debug::print("DEBUG: LoRa::Receive::ACK router=");
					Debug::print(router1);
					Debug::print(" terminal=");
					Debug::println(terminal);

					std::memcpy(content + sizeof terminal, &terminal, sizeof terminal);
					LoRa.beginPacket();
					LoRa.write(PacketType(PACKET_ACK));
					LoRa.write(Device(router1));
					LORA::Send::payload("ACK+", content + sizeof terminal, sizeof content - sizeof terminal);
					LoRa.endPacket();
				}
			}
		#endif

		static void packet(Time const now) {
			signed int const packet_size = LoRa.parsePacket();
			if (packet_size < 1) return;
			Debug::print("DEBUG: LORA::Receive::packet packet_size ");
			Debug::println(packet_size);
			#if defined(ENABLE_GATEWAY)
				Receive::last_time = now;
			#endif
			PacketType packet_type;
			if (LoRa.readBytes(&packet_type, sizeof packet_type) != sizeof packet_type) return;
			switch (packet_type) {
			case PACKET_TIME:
				Debug::println("DEBUG: LORA::Receive::packet TIME");
				TIME(packet_size);
				break;
			case PACKET_ASKTIME:
				Debug::println("DEBUG: LORA::Receive::packet ASKTIME");
				ASKTIME(packet_size);
				break;
			case PACKET_SEND:
				Debug::println("DEBUG: LORA::Receive::packet SEND");
				SEND(packet_size);
				break;
			case PACKET_ACK:
				Debug::println("DEBUG: LORA::Receive::packet ACK");
				ACK(packet_size);
				break;
			default:
				COM::print("LoRa: incorrect packet type: ");
				COM::println(packet_type);
			}

			/* add entropy to RNG */
			unsigned long int const microseconds = micros() ^ now;
			RNG.stir((uint8_t const *)&microseconds, sizeof microseconds, 8);
		}
	}

	static bool initialize(void) {
		SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
		LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

		#if !defined(ENABLE_GATEWAY)
			size_t const N = sizeof router_topology / sizeof *router_topology;
			size_t i = 0;
			for (size_t i = 0;; ++i) {
				if (i >= N) {
					last_receiver = 0;
					break;
				}
				if (router_topology[i][1] == Device(DEVICE_ID)) {
					last_receiver = router_topology[i][0];
					break;
				}
				++i;
			}
		#endif

		#if defined(ENABLE_GATEWAY)
			for (size_t i = 0; i < NUMBER_OF_DEVICES; ++i)
				LORA::Receive::last_serial[i] = 0;
		#endif

		if (LoRa.begin(LORA_BAND) == 1) {
			any_println("LoRa initialized");
			return true;
		} else {
			any_println("LoRa uninitialized");
			return false;
		}
	}
}

/* ************************************************************************** */

static bool setup_error = false;

void setup(void) {
	setup_error = false;
	LED::initialize();
	COM::initialize();
	OLED::initialize();

	#if defined(CPU_FREQUENCY)
		setCpuFrequencyMhz(CPU_FREQUENCY);
	#endif

	if (!setup_error)
		setup_error = !RTC::initialize();

	if (!setup_error)
		setup_error = !SD_CARD::initialize();

	if (!setup_error) {
		WIFI::initialize();
		setup_error = !LORA::initialize();
	}

	if (!setup_error) {
		Synchronize::initialize();
		AskTime::initialize();
		setup_error = !Measure::initialize();
	}

	if (!setup_error)
		Sender::initialize();

	OLED::display();

	#if !defined(ENABLE_SLEEP)
		LORA::Send::ASKTIME();
	#endif
}

void loop(void) {
	if (setup_error) {
		LED::flash();
		return;
	}
	Time const now = millis();
	LORA::Receive::packet(now);
	#if defined(ENABLE_GATEWAY) && defined(REBOOT_TIMEOUT)
		if (now - LORA::Receive::last_time > REBOOT_TIMEOUT)
			esp_restart();
	#endif
	WIFI::loop();
	Schedules::tick();
	RNG.loop();
	OLED::check_switch();
}

/* ************************************************************************** */
