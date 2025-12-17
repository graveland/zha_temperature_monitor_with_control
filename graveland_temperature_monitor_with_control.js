const {identify} = require('zigbee-herdsman-converters/lib/modernExtend');
const ota = require('zigbee-herdsman-converters/lib/ota');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const e = exposes.presets;
const ea = exposes.access;
const fs = require('fs');
const path = require('path');

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
};

const definition = {
    zigbeeModel: ['Temperature Monitor'],
    model: 'Temperature Monitor',
    vendor: 'graveland',
    description: 'Temperature monitor with freeze protection switch control (mains powered)',
    extend: [
        identify(),
    ],
    exposes: [
        // Use Thermostat cluster for temperature and setpoints
        e.climate()
            .withSetpoint('occupied_heating_setpoint', -20, 80, 0.5)
            .withSetpoint('occupied_cooling_setpoint', -20, 80, 0.5)
            .withLocalTemperature(),
        // Reboot switch on endpoint 10
        e.switch_().withEndpoint('reboot'),
        // Reset count from diagnostics cluster
        e.numeric('reset_count', ea.STATE).withDescription('Number of device resets'),
    ],
    fromZigbee: [
        fz.thermostat,
        fz.on_off,
        fzLocal.reset_count,
    ],
    toZigbee: [
        tz.thermostat_occupied_heating_setpoint,
        tz.thermostat_occupied_cooling_setpoint,
        tz.on_off,
    ],
    endpoint: (device) => {
        return {reboot: 10};
    },
    ota: {
        isUpdateAvailable: async (device, logger, data = null) => {
            // Read custom index.json directly
            const indexPath = path.join(data.otaPath || 'data/ota', 'index.json');
            logger.info(`Reading OTA index from: ${indexPath}`);

            try {
                const indexContent = fs.readFileSync(indexPath, 'utf8');
                const images = JSON.parse(indexContent);
                logger.info(`Found ${images.length} images in index`);

                // Get current version from device
                const endpoint = device.getEndpoint(1);
                await endpoint.read('genOta', ['currentFileVersion']);
                const currentVersion = endpoint.getClusterAttributeValue('genOta', 'currentFileVersion');
                logger.info(`Device current version: ${currentVersion}`);

                // Find matching image with highest version
                const manufacturerCode = device.manufacturerID;
                const imageType = device.modelID === 'Temperature Monitor' ? 0x5678 : null;

                const matchingImages = images.filter(img =>
                    img.manufacturerCode === manufacturerCode &&
                    img.imageType === imageType &&
                    img.fileVersion > currentVersion
                );

                logger.info(`Matching images: ${JSON.stringify(matchingImages)}`);

                if (matchingImages.length > 0) {
                    const latestImage = matchingImages.reduce((prev, current) =>
                        (prev.fileVersion > current.fileVersion) ? prev : current
                    );
                    return {
                        available: true,
                        currentFileVersion: currentVersion,
                        otaFileVersion: latestImage.fileVersion,
                    };
                }

                return {available: false, currentFileVersion: currentVersion};
            } catch (error) {
                logger.error(`Error reading OTA index: ${error.message}`);
                // Fallback to default behavior
                return ota.isUpdateAvailable(device, logger, data, {
                    imageBlockResponseDelay: 500,
                    useIndexOverride: true,
                });
            }
        },
        updateToLatest: async (device, logger, onProgress) => {
            return ota.updateToLatest(device, logger, onProgress, {
                imageBlockResponseDelay: 500,
                useIndexOverride: true,
            });
        },
    },
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
