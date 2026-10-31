#ifndef BME280_H
#define BME280_H



// ==== values to use ==== //
extern int32_t temperature_final;
extern int32_t pressure_final;
extern int32_t humidity_final;

void BME280_start(void);
void BME280_process(void);  

#endif /* BME280_H */
