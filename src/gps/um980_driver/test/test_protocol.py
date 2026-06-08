from um980_driver.protocol import (
    append_checksum,
    coordinate_to_decimal,
    parse_sentence,
    parse_unicore_sentence,
    RTCM3StreamParser,
    rtcm3_crc24q,
    unicore_crc32_hex,
    validate_checksum,
    validate_unicore_checksum,
)


def test_append_and_validate_checksum():
    sentence = append_checksum("GPGGA,123519.05,3723.2475,N,12700.1234,E,4,24,0.62,54.2,M,19.5,M,0.4,0000")

    assert validate_checksum(sentence)
    parsed = parse_sentence(sentence)
    assert parsed.talker == "GP"
    assert parsed.sentence_type == "GGA"


def test_coordinate_to_decimal():
    assert round(coordinate_to_decimal("3723.2475", "N"), 8) == 37.38745833
    assert round(coordinate_to_decimal("12700.1234", "E"), 8) == 127.00205667
    assert round(coordinate_to_decimal("3723.2475", "S"), 8) == -37.38745833


def test_parse_unicore_sentence():
    content = "BESTNAVA,COM1,0,0,FINESTEERING,2190,292957.000,02000800,0000,16636;SOL_COMPUTED,NARROW_FLOAT,37.1,127.2,48.3,23.0,WGS84"
    line = f"#{content}*{unicore_crc32_hex(content)}"

    assert validate_unicore_checksum(line)
    parsed = parse_unicore_sentence(line)

    assert parsed.log_type == "BESTNAVA"
    assert parsed.header_fields[1] == "COM1"
    assert parsed.body_fields[1] == "NARROW_FLOAT"


def test_rtcm3_stream_parser_extracts_message_id():
    payload = bytes([1074 >> 4, (1074 & 0x0F) << 4, 0x07])
    header = bytes([0xD3, 0x00, len(payload)])
    crc = rtcm3_crc24q(header + payload)
    packet = header + payload + crc.to_bytes(3, "big")

    messages = RTCM3StreamParser().parse(packet)

    assert len(messages) == 1
    assert messages[0].message_id == 1074
    assert messages[0].station_id == 7
