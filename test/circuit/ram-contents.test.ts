/**
 * `params.contents` on the RAM kinds (coordinator ruling 2026-11-13): a
 * power-on preload, shaped like the EEPROMs' image. These cases pin the
 * field's shape and its two readers: the lenient decoder the engines load
 * with (it mirrors `eepromImageBytes`: it never throws) and the strict check
 * an editor runs before it saves a value.
 */
import { describe, expect, it } from "vitest";
import {
  RAM_CONTENTS_SHAPE,
  RAM_KINDS,
  encodeRamContents,
  isEepromKind,
  isRamKind,
  ramContentsError,
  ramPreloadWords,
} from "../../src/circuit/types.js";

const b64 = (bytes: readonly number[]): string => btoa(String.fromCharCode(...bytes));
const COUNT_UP = Array.from({ length: 16 }, (_, i) => i);

describe("RAM kinds", () => {
  it("names the 74LS189 and no EEPROM", () => {
    expect([...RAM_KINDS]).toEqual(["74ls189"]);
    expect(isRamKind("74ls189")).toBe(true);
    // A RAM's contents are a preload and an EEPROM's are its image: the two sets must not overlap.
    expect(isRamKind("28c16")).toBe(false);
    expect(isEepromKind("74ls189")).toBe(false);
  });

  it("gives the 74LS189 sixteen words of four bits", () => {
    // TI SDLS113: a 64-bit RAM organised as 16 words of 4 bits.
    expect(RAM_CONTENTS_SHAPE["74ls189"]).toEqual({ words: 16, bits: 4 });
  });
});

describe("ramPreloadWords (the engines' decoder)", () => {
  it("reads no preload from an absent or empty value", () => {
    // null is "no preload": the RAM then starts exactly as it did before the field existed.
    expect(ramPreloadWords("74ls189", undefined)).toBeNull();
    expect(ramPreloadWords("74ls189", "")).toBeNull();
  });

  it("reads one word per byte, address 0 first", () => {
    // Bytes 0..15 are the words at addresses 0..15, in true sense (the /O pins show the complement).
    expect(ramPreloadWords("74ls189", b64(COUNT_UP))).toEqual(COUNT_UP);
  });

  it("leaves the addresses a short image does not reach blank, as the EEPROM's 0xFF fill does", () => {
    // Three bytes given: addresses 3..15 keep the RAM's blank word, 0xF (classic's power-on value).
    expect(ramPreloadWords("74ls189", b64([1, 2, 3]))).toEqual([1, 2, 3, ...new Array<number>(13).fill(0xf)]);
  });

  it("drops the bytes past the last address, as the EEPROM's decoder does", () => {
    // Twenty bytes: only the first sixteen have an address.
    expect(ramPreloadWords("74ls189", b64([...COUNT_UP, 9, 9, 9, 9]))).toEqual(COUNT_UP);
  });

  it("keeps the low four bits of a byte that holds more", () => {
    // 0xA5 does not fit a 4-bit word; the chip has four data bits, so the low nibble (5) is what it stores.
    expect(ramPreloadWords("74ls189", b64([0xa5]))?.[0]).toBe(0x5);
  });

  it("reads a value that is not base64 as a blank image, as the EEPROM's decoder does", () => {
    // atob throws on '!': every word stays blank (0xF). It is still a preload, so it is applied at power-up.
    expect(ramPreloadWords("74ls189", "!!!not base64!!!")).toEqual(new Array<number>(16).fill(0xf));
    // A number is read through String(), as classic reads an EEPROM's contents: "7" is not valid base64 either.
    expect(ramPreloadWords("74ls189", 7)).toEqual(new Array<number>(16).fill(0xf));
  });
});

describe("ramContentsError (an editor's check)", () => {
  it("accepts no value, and exactly sixteen 4-bit words", () => {
    expect(ramContentsError("74ls189", undefined)).toBeNull();
    expect(ramContentsError("74ls189", "")).toBeNull();
    expect(ramContentsError("74ls189", b64(COUNT_UP))).toBeNull();
  });

  it("refuses a value that is not a string", () => {
    expect(ramContentsError("74ls189", 7)).toMatch(/base64 string/);
  });

  it("refuses a string that is not base64", () => {
    expect(ramContentsError("74ls189", "!!!")).toMatch(/not valid base64/);
  });

  it("refuses an image of the wrong size, naming both counts", () => {
    // 15 and 17 bytes are both wrong: the image is one byte per word, sixteen words.
    expect(ramContentsError("74ls189", b64(COUNT_UP.slice(0, 15)))).toMatch(/15 byte\(s\).*16/);
    expect(ramContentsError("74ls189", b64([...COUNT_UP, 0]))).toMatch(/17 byte\(s\).*16/);
  });

  it("refuses a word wider than four bits, naming its address", () => {
    const words = [...COUNT_UP];
    words[9] = 0x10; // one past the largest 4-bit value
    expect(ramContentsError("74ls189", b64(words))).toMatch(/address 9.*0x10/);
  });
});

describe("encodeRamContents", () => {
  it("round-trips through the decoder", () => {
    expect(ramPreloadWords("74ls189", encodeRamContents("74ls189", COUNT_UP))).toEqual(COUNT_UP);
    // The encoded form is plain base64 of the sixteen bytes 00..0F.
    expect(encodeRamContents("74ls189", COUNT_UP)).toBe("AAECAwQFBgcICQoLDA0ODw==");
  });

  it("throws on a value the strict check would refuse", () => {
    expect(() => encodeRamContents("74ls189", [1, 2, 3])).toThrow(RangeError);
    expect(() => encodeRamContents("74ls189", [...COUNT_UP.slice(0, 15), 16])).toThrow(RangeError);
  });
});
