const {identify, deviceAddCustomCluster, numeric} = require('zigbee-herdsman-converters/lib/modernExtend');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const e = exposes.presets;
const ea = exposes.access;

const fzLocal = {
    reset_count: {
        cluster: 'haDiagnostic',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data.numberOfResets !== undefined) {
                return {reset_count: msg.data.numberOfResets};
            }
        },
    },
    rejoin_count: {
        cluster: 'haDiagnostic',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            // Manufacturer-specific 0xC001; zigbee-herdsman may key unknown
            // attributes by hex string or by decimal value.
            for (const key of ['0xc001', '0xC001', '49153']) {
                if (msg.data[key] !== undefined) {
                    return {rejoin_count: msg.data[key]};
                }
            }
        },
    },
};

// Counters from the firmware's diagnostics cluster, attribute id = index. They show how reports from a bound
// temperature sensor reach this device: raw frame seen -> callback delivered -> report stored.
const diagAttributes = [
    ['rawTotal', 'diag_raw_total', 'ZCL frames the stack passed to the raw handler'],
    ['rawReports', 'diag_raw_reports', 'Temperature Measurement report frames received'],
    ['rawReportSrc', 'diag_raw_report_src', 'Short address of the sender of the last temperature report frame'],
    ['reportCb', 'diag_report_cb', 'Report callbacks delivered to the application, any cluster'],
    ['reportCbStatus', 'diag_report_cb_status', 'Status of the last report callback'],
    ['reportStored', 'diag_report_stored', 'Reports accepted and mirrored as outdoor temperature'],
    ['otherCb', 'diag_other_cb', 'Action callbacks with no handler'],
    ['otherCbId', 'diag_other_cb_id', 'Id of the last unhandled action callback'],
    ['resetReason', 'diag_reset_reason', 'Reset reason of this boot (1 power on, 3 software, 4 panic, 5 interrupt watchdog, 6 task watchdog, 7 other watchdog, 9 brownout)'],
    ['crumbZbPrev', 'diag_crumb_zb_prev', 'Last Zigbee callback step before the previous reset (1 raw frame, 2 report in, 3 validated, 4 stored, 5 done, 7 read response, 8 write, 9 other, 10 signal)'],
    ['crumbTimerPrev', 'diag_crumb_timer_prev', 'Last esp_timer step before the previous reset (20 temperature timer, 21 sensors read, 22 control, 23 report, 24 diagnostics, 25 done, 26 remote temperature, 30 watchdog, 40 switch sync)'],
    ['minFreeHeapKb', 'diag_min_free_heap_kb', 'Lowest free heap since boot (KB)'],
    ['stackFreeZb', 'diag_stack_free_zb', 'Lowest free bytes of the Zigbee task stack'],
    ['stackFreeTimer', 'diag_stack_free_timer', 'Lowest free bytes of the esp_timer task stack'],
    ['restartCausePrev', 'diag_restart_cause_prev', 'Why the previous run called esp_restart(), 0 if it did not (1 no readings, 2 stuck temperature, 3 not joined, 4 reboot switch, 5 OTA done, 99 other)'],
];

const definition = {
    zigbeeModel: ['Temperature Monitor'],
    model: 'Temperature Monitor',
    vendor: 'graveland',
    description: 'Temperature monitor with freeze protection switch control (mains powered)',
    extend: [
        identify(),
        deviceAddCustomCluster('graveland_diag', {
            name: 'graveland_diag',
            ID: 0xFC11,
            attributes: Object.fromEntries(diagAttributes.map(([attribute], id) => [attribute, {name: attribute, ID: id, type: 0x21}])),
            commands: {},
            commandsResponse: {},
        }),
        ...diagAttributes.map(([attribute, name, description]) => numeric({
            name,
            cluster: 'graveland_diag',
            attribute,
            description,
            access: 'STATE_GET',
        })),
    ],
    exposes: [
        // Use Thermostat cluster for temperature and setpoints
        e.climate()
            .withSetpoint('occupied_heating_setpoint', -20, 80, 0.5)
            .withSetpoint('occupied_cooling_setpoint', -20, 80, 0.5)
            .withLocalTemperature(),
        // Temperature reported by a sensor bound to this device, mirrored from its Temperature Measurement reports
        e.numeric('outdoor_temperature', ea.STATE_GET).withUnit('°C').withDescription('Temperature reported by a bound sensor'),
        // Reboot switch on endpoint 10
        e.switch_().withState('state', true).withEndpoint('reboot'),
        // Reset count from diagnostics cluster
        e.numeric('reset_count', ea.STATE).withDescription('Number of device resets'),
        e
            .switch_()
            .withState('state', true)
            .withEndpoint('rejoin')
            .withDescription('Leave the network and re-steer (send TOGGLE)'),
        e.numeric('rejoin_count', ea.STATE).withDescription('Number of forced re-joins'),
    ],
    fromZigbee: [
        fz.thermostat,
        fz.on_off,
        fzLocal.reset_count,
        fzLocal.rejoin_count,
    ],
    toZigbee: [
        tz.thermostat_occupied_heating_setpoint,
        tz.thermostat_occupied_cooling_setpoint,
        tz.thermostat_outdoor_temperature,
        tz.on_off,
    ],
    endpoint: (device) => {
        return {reboot: 10, rejoin: 11};
    },
    ota: true,
    meta: {
        // This device uses binding to control switches, so we don't expose the on/off cluster directly
        // The on/off client cluster is used internally to send commands to bound devices
    },
    configure: async (device, coordinatorEndpoint) => {
        const endpoint = device.getEndpoint(1);

        // Bind thermostat cluster for reporting
        await endpoint.bind('hvacThermostat', coordinatorEndpoint);

        // Configure temperature reporting via thermostat's localTemp
        await endpoint.configureReporting('hvacThermostat', [
            {
                attribute: 'localTemp',
                minimumReportInterval: 5,      // Report at least every 5 seconds
                maximumReportInterval: 300,    // Report at least every 5 minutes
                reportableChange: 50,          // Report on 0.5°C change (value is in 0.01°C units)
            }
        ]);

        // Read thermostat setpoints
        await endpoint.read('hvacThermostat', ['localTemp', 'occupiedHeatingSetpoint', 'occupiedCoolingSetpoint']);

        // Configure reboot switch endpoint
        const rebootEndpoint = device.getEndpoint(10);
        if (rebootEndpoint) {
            await rebootEndpoint.bind('genOnOff', coordinatorEndpoint);
            await rebootEndpoint.read('genOnOff', ['onOff']);
        }

        // Read reset count from diagnostics cluster
        await endpoint.read('haDiagnostic', ['numberOfResets']);
    },
};

module.exports = definition;
